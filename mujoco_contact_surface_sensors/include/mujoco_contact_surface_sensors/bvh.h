/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: David Leins */

#pragma once

#ifdef _MSC_VER
#define ALIGN(x) __declspec(align(x))
#define MALLOC64(x) ((x) == 0 ? 0 : _aligned_malloc((x), 64))
#define FREE64(x) _aligned_free(x)
#else
#define ALIGN(x) __attribute__((aligned(x)))
#define MALLOC64(x) ((x) == 0 ? 0 : aligned_alloc(64, (x)))
#define FREE64(x) free(x)
#endif

// bin count for binned BVH building
#define BINS 8

#include <emmintrin.h>

#include <mujoco_contact_surface_sensors/float3.h>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>
#include <chrono>

#include <drake/geometry/query_results/contact_surface.h>
#include <drake/geometry/proximity/triangle_surface_mesh.h>

namespace mujoco_contact_surface_sensors {
using namespace drake;
using namespace drake::geometry;

// 16 bytes structure is optimized for GPU memory access (16 bytes = single read memory operation on GPU)
ALIGN(16) struct Intersection
{
	float t; // distance along ray
	float u, v; // barycentric coordinates
	uint bvh_triangle; // bvh (12 bit) and traingle (20 bit) index
};

struct Triangle
{
	union
	{
		float3 vertex0;
		__m128 v0;
	};
	union
	{
		float3 vertex1;
		__m128 v1;
	};
	union
	{
		float3 vertex2;
		__m128 v2;
	};
	union
	{
		float3 centroid;
		__m128 centroid4;
	};
};

ALIGN(64) struct Ray
{
	Ray() { d0.O4 = d1.D4 = d2.rD4 = _mm_set1_ps(1); }
	union
	{
		struct
		{
			float3 O;
			float dummy1;
		} data;
		__m128 O4;
	} d0;
	union
	{
		struct
		{
			float3 D;
			float dummy2;
		} data;
		__m128 D4;
	} d1;
	union
	{
		struct
		{
			float3 rD;
			float dummy3;
		} data;
		__m128 rD4;
	} d2;

	Intersection hit; // total ray size: 64 bytes
};

struct AABB
{
	float3 bmin = 1e30f;
	float3 bmax = -1e30f;
	float area()
	{
		float3 size = bmax - bmin;
		return size.x * size.y + size.x * size.z + size.y * size.z;
	}
	void grow(float3 p)
	{
		bmin = fminf(bmin, p);
		bmax = fmaxf(bmax, p);
	}
	void grow(AABB &b)
	{
		if (b.bmin.x != 1e30f) {
			grow(b.bmin);
			grow(b.bmax);
		}
	}
};

struct BVHNode
{
	union
	{
		struct
		{
			float3 aabbMin;
			uint left_first;
		} data;
		__m128 aabbMin4;
	} d0;
	union
	{
		struct
		{
			float3 aabbMax;
			uint triangle_count;
		} data;
		__m128 aabbMax4;
	} d1;
	bool is_leaf() { return d1.data.triangle_count > 0; }
	float calculateNodeCost()
	{
		float3 size = d1.data.aabbMax - d0.data.aabbMin;
		return (size.x * size.y + size.x * size.z + size.y * size.z) * d1.data.triangle_count;
	}
};

class BVH
{
public:
	BVH() = default;
	BVH(std::shared_ptr<ContactSurface<double>> surface);
	~BVH();

	void build();
	void intersect(Ray &ray, uint blas_idx);

	AABB bounds; // in world space
	std::shared_ptr<drake::geometry::ContactSurface<double>> surface;

private:
	void subdivide(uint node_index, uint depth, uint &node_ptr, float3 &centroid_min, float3 &centroid_max);
	void updateNodeBounds(uint node_index, float3 &centroid_min, float3 &centroid_max);
	float findBestSplitPlane(BVHNode &node, int &axis, int &split_pos, float3 &centroid_min, float3 &centroid_max);

	std::shared_ptr<BVHNode> bvh_node;
	Triangle *tri = 0;
	uint *tri_idx = 0;
	uint nodes_used;
	uint triangle_count;
};

struct TLASNode
{
	union
	{
		struct
		{
			float dummy1[3];
			uint left_right;
		} data1;
		struct
		{
			float dummy3[3];
			unsigned short left, right;
		} data2;
		float3 aabbMin;
		__m128 aabbMin4;
	} d0;

	union
	{
		struct
		{
			float dummy2[3];
			uint blas;
		} data;
		float3 aabbMax;
		__m128 aabbMax4;
	} d1;
	bool is_leaf() { return d0.data1.left_right == 0; }
};

ALIGN(64) class TLAS
{
public:
	TLAS() = default;
	TLAS(BVH *bhv_list, int num_bvhs);
	~TLAS();

	void build();
	void intersect(Ray &ray);

	BVH *blas = 0;

private:
	TLASNode *tlas_node = 0;
	uint nodes_used, blas_count;
	uint *node_idx = 0;

	// fast agglomerative clustering functionality
	int findBestMatch(int N, int A);
};

} // namespace mujoco_contact_surface_sensors
