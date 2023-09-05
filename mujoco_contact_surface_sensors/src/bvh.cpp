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

#include <mujoco_contact_surface_sensors/bvh.h>
#include <iostream>

#include <drake/common/eigen_types.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/geometry/proximity/triangle_surface_mesh.h>
#include <drake/geometry/proximity/triangle_surface_mesh_field.h>

#include <smmintrin.h>

namespace mujoco_ros::contact_surfaces::sensors {

std::unique_ptr<TriangleSurfaceMesh<double>> GenerateMesh()
{
	std::vector<Eigen::Vector3d> vertices = { { 0., 0., 0. }, { 1., 0., 0. }, { 1., 1., 0. }, { 0., 1., 0. } };
	std::vector<SurfaceTriangle> faces{ { 0, 1, 2 }, { 2, 3, 0 } };
	return std::make_unique<TriangleSurfaceMesh<double>>(std::move(faces), std::move(vertices));
}

std::unique_ptr<MeshFieldLinear<double, TriangleSurfaceMesh<double>>> MakeField(std::vector<double> &&e_values,
                                                                                const TriangleSurfaceMesh<double> &mesh)
{
	auto tri_mesh = GenerateMesh();
	TriangleSurfaceMeshFieldLinear<double, double> field(std::vector<double>(e_values), tri_mesh.get());

	std::vector<Eigen::Vector3d> e_grad;
	for (int t = 0; t < tri_mesh->num_elements(); ++t) {
		e_grad.push_back(field.EvaluateGradient(t));
	}
	return std::make_unique<MeshFieldLinear<double, TriangleSurfaceMesh<double>>>(std::move(e_values), &mesh,
	                                                                              std::move(e_grad));
}

drake::geometry::ContactSurface<double> GenerateContactSurface()
{
	auto id_M         = GeometryId::get_new_id();
	auto id_N         = GeometryId::get_new_id();
	auto surface_mesh = GenerateMesh();

	std::vector<double> e_values = { 0., 1., 2., 3. };
	std::unique_ptr<MeshFieldLinear<double, TriangleSurfaceMesh<double>>> e_field =
	    MakeField(std::move(e_values), *surface_mesh);

	drake::geometry::ContactSurface<double> contact_surface(id_M, id_N, std::move(surface_mesh), std::move(e_field));
	return contact_surface;
}

void IntersectTriangle(Ray &ray, const Triangle &tri, const uint bvh_triangle)
{
	// Moeller-Trumbore algorithm
	const float3 edge1 = tri.vertex1 - tri.vertex0;
	const float3 edge2 = tri.vertex2 - tri.vertex0;
	const float3 h     = cross(ray.d1.data.D, edge2);
	const float a      = dot(edge1, h);
	if (std::fabs(a) < 1e-5f)
		return; // parallel to triangle
	const float f  = 1.0f / a;
	const float3 s = ray.d0.data.O - tri.vertex0;
	const float u  = f * dot(s, h);
	if (u < 0.0f || u > 1.0f)
		return;
	const float3 q = cross(s, edge1);
	const float v  = f * dot(ray.d1.data.D, q);
	if (v < 0.0f || u + v > 1.0f)
		return;
	const float t = f * dot(edge2, q);
	if (t < ray.hit.t && t > 0.0f) {
		ray.hit.t            = t;
		ray.hit.u            = u;
		ray.hit.v            = v;
		ray.hit.bvh_triangle = bvh_triangle;
	}
}

float IntersectAABB_SSE(const Ray &ray, const __m128 &bmin4, const __m128 &bmax4)
{
	static __m128 mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0));
	__m128 t1           = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmin4, mask4), ray.d0.O4), ray.d2.rD4);
	__m128 t2           = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(bmax4, mask4), ray.d0.O4), ray.d2.rD4);
	__m128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
	float tmax = std::min(vmax4[0], std::min(vmax4[1], vmax4[2]));
	float tmin = std::max(vmin4[0], std::max(vmin4[1], vmin4[2]));
	if (tmax >= tmin && tmin < ray.hit.t && tmax > 0)
		return tmin;
	else
		return 1e30f;
}

BVH::BVH(std::shared_ptr<drake::geometry::ContactSurface<double>> contact_surface)
{
	surface        = contact_surface;
	triangle_count = surface->tri_mesh_W().num_elements();
	tri            = new Triangle[triangle_count];

	for (int i = 0; i < triangle_count; i++) {
		const auto &drake_tri = surface->tri_mesh_W().element(i);

		// cast double vector to float, and store the data which we optimized our code for
		Eigen::Map<Eigen::Vector3f>(&tri[i].vertex0.cell[0], 3) =
		    surface->tri_mesh_W().vertex(drake_tri.vertex(0)).cast<float>();
		Eigen::Map<Eigen::Vector3f>(&tri[i].vertex1.cell[0], 3) =
		    surface->tri_mesh_W().vertex(drake_tri.vertex(1)).cast<float>();
		Eigen::Map<Eigen::Vector3f>(&tri[i].vertex2.cell[0], 3) =
		    surface->tri_mesh_W().vertex(drake_tri.vertex(2)).cast<float>();
	}

#ifdef _MSC_VER
	bvh_node = reset(static_cast<BVHNode *>(_aligned_malloc(sizeof(BVHNode) * triangle_count * 2, 64)), &_aligned_free);
#else
	bvh_node.reset(static_cast<BVHNode *>(aligned_alloc(64, sizeof(BVHNode) * triangle_count * 2)), &free);
#endif
	tri_idx = new uint[triangle_count];
	build();
}

BVH::~BVH()
{
	if (bvh_node.use_count() == 1) { // last reference is being deleted
		delete[] tri;
		delete[] tri_idx;
	}
}

void BVH::intersect(Ray &ray, uint blas_idx)
{
	BVHNode *node  = &bvh_node.get()[0], *stack[64];
	uint stack_ptr = 0;
	while (1) {
		if (node->is_leaf()) {
			for (uint i = 0; i < node->d1.data.triangle_count; i++) {
				uint blas_triangle = (blas_idx << 20) + tri_idx[node->d0.data.left_first + i];
				IntersectTriangle(ray, tri[tri_idx[node->d0.data.left_first + i]], blas_triangle);
			}
			if (stack_ptr == 0)
				break;
			else
				node = stack[--stack_ptr];
			continue;
		}
		BVHNode *child1 = &bvh_node.get()[node->d0.data.left_first];
		BVHNode *child2 = &bvh_node.get()[node->d0.data.left_first + 1];
#ifdef USE_SSE
		float dist1 = IntersectAABB_SSE(ray, child1->d0.aabbMin4, child1->d1.aabbMax4);
		float dist2 = IntersectAABB_SSE(ray, child2->d0.aabbMin4, child2->d1.aabbMax4);
#else
		float dist1 = IntersectAABB(ray, child1->d0.data.aabbMin, child1->d1.data.aabbMax);
		float dist2 = IntersectAABB(ray, child2->d0.data.aabbMin, child2->d1.data.aabbMax);
#endif
		if (dist1 > dist2) {
			std::swap(child1, child2);
			std::swap(dist1, dist2);
		}
		if (dist1 == 1e30f) { // missed both children
			if (stack_ptr == 0)
				break;
			else
				node = stack[--stack_ptr];
		} else { // process child1 and put child2 on stack
			node = child1;
			if (dist2 != 1e30f)
				stack[stack_ptr++] = child2;
		}
	}
}

void BVH::build()
{
	// reset node pool
	nodes_used = 2;
	memset(bvh_node.get(), 0, sizeof(BVHNode) * triangle_count * 2);
	// initialize triangle indices and calculate traingle centroids for partitioning
	for (int i = 0; i < triangle_count; i++) {
		tri_idx[i]      = i;
		tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * 0.3333f;
	}
	// assign all triangles to root node
	BVHNode &root               = bvh_node.get()[0];
	root.d0.data.left_first     = 0;
	root.d1.data.triangle_count = triangle_count;
	float3 centroid_min, centroid_max;
	updateNodeBounds(0, centroid_min, centroid_max);
	// subdivide recursively
	subdivide(0, 0, nodes_used, centroid_min, centroid_max);

	bounds      = AABB();
	bounds.bmin = bvh_node.get()[0].d0.data.aabbMin;
	bounds.bmax = bvh_node.get()[0].d1.data.aabbMax;
}

void BVH::subdivide(uint node_idx, uint depth, uint &node_ptr, float3 &centroid_min, float3 &centroid_max)
{
	BVHNode &node = bvh_node.get()[node_idx];
	// determine split axis using SAH
	int axis, split_pos;
	float split_cost = findBestSplitPlane(node, axis, split_pos, centroid_min, centroid_max);
	// terminate recursion
	float nosplit_cost = node.calculateNodeCost();
	if (split_cost >= nosplit_cost)
		return;

	// in-place partitioning
	int i       = node.d0.data.left_first;
	int j       = i + node.d1.data.triangle_count - 1;
	float scale = BINS / (centroid_max[axis] - centroid_min[axis]);
	while (i <= j) {
		int bin_idx = std::min(BINS - 1, (int)((tri[tri_idx[i]].centroid[axis] - centroid_min[axis]) * scale));
		if (bin_idx < split_pos)
			i++;
		else
			std::swap(tri_idx[i], tri_idx[j--]);
	}

	// abort split if one of the sides is empty
	int left_count = i - node.d0.data.left_first;
	if (left_count == 0 || left_count == node.d1.data.triangle_count)
		return;

	// create child nodes
	int left_child_idx  = node_ptr++;
	int right_child_idx = node_ptr++;

	bvh_node.get()[left_child_idx].d0.data.left_first      = node.d0.data.left_first;
	bvh_node.get()[left_child_idx].d1.data.triangle_count  = left_count;
	bvh_node.get()[right_child_idx].d0.data.left_first     = i;
	bvh_node.get()[right_child_idx].d1.data.triangle_count = node.d1.data.triangle_count - left_count;
	node.d0.data.left_first                                = left_child_idx;
	node.d1.data.triangle_count                            = 0;

	// recurse in both paths
	updateNodeBounds(left_child_idx, centroid_min, centroid_max);
	subdivide(left_child_idx, depth + 1, node_ptr, centroid_min, centroid_max);
	updateNodeBounds(right_child_idx, centroid_min, centroid_max);
	subdivide(right_child_idx, depth + 1, node_ptr, centroid_min, centroid_max);
}

float BVH::findBestSplitPlane(BVHNode &node, int &axis, int &split_pos, float3 &centroid_min, float3 &centroid_max)
{
	float best_cost = 1e30f;
	for (int a = 0; a < 3; a++) {
		float bounds_min = centroid_min[a];
		float bounds_max = centroid_max[a];

		// populate the bins
		float scale = BINS / (bounds_max - bounds_min);
		float left_count_area[BINS - 1], right_count_area[BINS - 1];
		int left_sum = 0, right_sum = 0;
#ifdef USE_SSE
		__m128 min4[BINS], max4[BINS];
		uint count[BINS];
		for (uint i = 0; i < BINS; i++)
			min4[i] = _mm_set1_ps(1e30f), max4[i] = _mm_set1_ps(-1e30f), count[i] = 0;
		for (uint i = 0; i < node.d1.data.triangle_count; i++) {
			Triangle &triangle = tri[tri_idx[node.d0.data.left_first + i]];
			int bin_idx        = std::max(std::min(BINS - 1, (int)((triangle.centroid[a] - bounds_min) * scale)), 0);
			count[bin_idx]++;
			min4[bin_idx] = _mm_min_ps(min4[bin_idx], triangle.v0);
			max4[bin_idx] = _mm_max_ps(max4[bin_idx], triangle.v0);
			min4[bin_idx] = _mm_min_ps(min4[bin_idx], triangle.v1);
			max4[bin_idx] = _mm_max_ps(max4[bin_idx], triangle.v1);
			min4[bin_idx] = _mm_min_ps(min4[bin_idx], triangle.v2);
			max4[bin_idx] = _mm_max_ps(max4[bin_idx], triangle.v2);
		}
		// gather data for the 7 planes between the 8 bins
		__m128 left_min4 = _mm_set_ps1(1e30f), right_min4 = left_min4;
		__m128 left_max4 = _mm_set_ps1(-1e30f), right_max4 = left_max4;
		for (int i = 0; i < BINS - 1; i++) {
			left_sum += count[i];
			right_sum += count[BINS - 1 - i];
			left_min4                      = _mm_min_ps(left_min4, min4[i]);
			right_min4                     = _mm_min_ps(right_min4, min4[BINS - 2 - i]);
			left_max4                      = _mm_max_ps(left_max4, max4[i]);
			right_max4                     = _mm_max_ps(right_max4, max4[BINS - 2 - i]);
			const __m128 le                = _mm_sub_ps(left_max4, left_min4);
			const __m128 re                = _mm_sub_ps(right_max4, right_min4);
			left_count_area[i]             = left_sum * (le[0] * le[1] + le[1] * le[2] + le[2] * le[0]);
			right_count_area[BINS - 2 - i] = right_sum * (re[0] * re[1] + re[1] * re[2] + re[2] * re[0]);
		}
#else
		struct Bin
		{
			AABB bounds;
			int triangle_count = 0;
		} bin[BINS];
		for (uint i = 0; i < node.d1.data.triangle_count; i++) {
			Triangle &triangle = tri[tri_idx[node.d0.data.left_first + i]];
			int bin_idx        = std::max(std::min(BINS - 1, (int)((triangle.centroid.cell[a] - bounds_min) * scale)), 0);
			bin[bin_idx].triangle_count++;
			bin[bin_idx].bounds.grow(triangle.vertex0);
			bin[bin_idx].bounds.grow(triangle.vertex1);
			bin[bin_idx].bounds.grow(triangle.vertex2);
		}
		// gather data for the 7 planes between the 8 bins
		AABB left_box, right_box;
		for (int i = 0; i < BINS - 1; i++) {
			left_sum += bin[i].triangle_count;
			left_box.grow(bin[i].bounds);
			left_count_area[i] = left_box.area() * left_sum;
			right_sum += bin[BINS - 1 - i].triangle_count;
			right_box.grow(bin[BINS - 1 - i].bounds);
			right_count_area[BINS - 2 - i] = right_box.area() * right_sum;
		}
#endif
		// calculate SAH cost for all 7 planes
		scale = (bounds_max - bounds_min) / BINS;
		for (int i = 0; i < BINS - 1; i++) {
			const float plane_cost = left_count_area[i] + right_count_area[i];
			if (plane_cost < best_cost) {
				best_cost = plane_cost;
				axis      = a;
				split_pos = i + 1;
			}
		}
	}
	return best_cost;
}

void BVH::updateNodeBounds(uint node_idx, float3 &centroid_min, float3 &centroid_max)
{
	BVHNode &node = bvh_node.get()[node_idx];
#ifdef USE_SSE
	__m128 min4 = _mm_set_ps1(1e30f), cmin4 = min4;
	__m128 max4 = _mm_set_ps1(-1e30f), cmax4 = max4;
	for (uint first = node.d0.data.left_first, i = 0; i < node.d1.data.triangle_count; i++) {
		Triangle &leaf = tri[tri_idx[first + i]];
		min4 = _mm_min_ps(min4, leaf.v0), max4 = _mm_max_ps(max4, leaf.v0);
		min4 = _mm_min_ps(min4, leaf.v1), max4 = _mm_max_ps(max4, leaf.v1);
		min4 = _mm_min_ps(min4, leaf.v2), max4 = _mm_max_ps(max4, leaf.v2);
		cmin4 = _mm_min_ps(cmin4, leaf.centroid4);
		cmax4 = _mm_max_ps(cmax4, leaf.centroid4);
	}
	__m128 mask4     = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0));
	node.d0.aabbMin4 = _mm_blendv_ps(node.d0.aabbMin4, min4, mask4);
	node.d1.aabbMax4 = _mm_blendv_ps(node.d1.aabbMax4, max4, mask4);
	centroid_min     = *(float3 *)&cmin4;
	centroid_max     = *(float3 *)&cmax4;
#else
	node.d0.data.aabbMin = float3(1e30f);
	node.d1.data.aabbMax = float3(-1e30f);
	centroid_min         = float3(1e30f);
	centroid_max         = float3(-1e30f);
	for (uint first = node.d0.data.left_first, i = 0; i < node.d1.data.triangle_count; i++) {
		uint leaf_tri_idx    = tri_idx[first + i];
		Triangle &leaf       = tri[leaf_tri_idx];
		node.d0.data.aabbMin = fminf(node.d0.data.aabbMin, leaf.vertex0);
		node.d0.data.aabbMin = fminf(node.d0.data.aabbMin, leaf.vertex1);
		node.d0.data.aabbMin = fminf(node.d0.data.aabbMin, leaf.vertex2);

		node.d1.data.aabbMax = fmaxf(node.d1.data.aabbMax, leaf.vertex0);
		node.d1.data.aabbMax = fmaxf(node.d1.data.aabbMax, leaf.vertex1);
		node.d1.data.aabbMax = fmaxf(node.d1.data.aabbMax, leaf.vertex2);

		centroid_min = fminf(centroid_min, leaf.centroid);
		centroid_max = fmaxf(centroid_max, leaf.centroid);
	}
#endif
}

TLAS::TLAS(BVH *bvh_list, int num_blas)
{
	// copy a pointer to the array of bottom level accstructs
	blas       = bvh_list;
	blas_count = num_blas;
	// allocate TLAS nodes
#ifdef _MSC_VER
	tlas_node = (TLASNode *)_aligned_malloc(sizeof(TLASNode) * blas_count * 2, 64);
#else
	tlas_node = (TLASNode *)aligned_alloc(64, sizeof(TLASNode) * blas_count * 2);
#endif
	node_idx   = new uint[num_blas];
	nodes_used = 2;
}

TLAS::~TLAS()
{
#ifdef _MSC_VER
	_aligned_free(tlas_node);
#else
	free(tlas_node);
#endif
	delete[] node_idx;
}

int TLAS::findBestMatch(int N, int A)
{
	// find BLAS B that, when joined with A, forms the smallest AABB
	float smallest = 1e30f;
	int best_B     = -1;
	for (int B = 0; B < N; B++)
		if (B != A) {
			float3 bmax        = fmaxf(tlas_node[node_idx[A]].d1.aabbMax, tlas_node[node_idx[B]].d1.aabbMax);
			float3 bmin        = fminf(tlas_node[node_idx[A]].d0.aabbMin, tlas_node[node_idx[B]].d0.aabbMin);
			float3 size        = bmax - bmin;
			float surface_area = size.x * size.y + size.x * size.z + size.y * size.z;
			if (surface_area < smallest)
				smallest = surface_area, best_B = B;
		}
	return best_B;
}

void TLAS::build()
{
	// assign TLAS leaf node to each BLAS
	nodes_used = 1;
	for (uint i = 0; i < blas_count; i++) {
		node_idx[i]                                 = nodes_used;
		tlas_node[nodes_used].d0.aabbMin            = blas[i].bounds.bmin;
		tlas_node[nodes_used].d1.aabbMax            = blas[i].bounds.bmax;
		tlas_node[nodes_used].d1.data.blas          = i;
		tlas_node[nodes_used++].d0.data1.left_right = 0; // makes it a leaf
	}
	// agglomerative clustering
	int node_indices = blas_count;
	int A            = 0;
	int B            = findBestMatch(node_indices, A);

	while (node_indices > 1) {
		int C = findBestMatch(node_indices, B);
		if (A == C) {
			int node_idx_a = node_idx[A], node_idx_b = node_idx[B];
			TLASNode &node_a = tlas_node[node_idx_a], &node_b = tlas_node[node_idx_b];
			TLASNode &new_node           = tlas_node[nodes_used];
			new_node.d0.data1.left_right = node_idx_a + (node_idx_b << 16);
			new_node.d0.aabbMin          = fminf(node_a.d0.aabbMin, node_b.d0.aabbMin);
			new_node.d1.aabbMax          = fmaxf(node_a.d1.aabbMax, node_b.d1.aabbMax);
			node_idx[A]                  = nodes_used++;
			node_idx[B]                  = node_idx[node_indices - 1];
			B                            = findBestMatch(--node_indices, A);
		} else {
			A = B;
			B = C;
		}
	}
	// copy last remaining node to root
	tlas_node[0] = tlas_node[node_idx[A]];
}

void TLAS::intersect(Ray &ray)
{
	// calculate reciprocal direction for faster AABB intersection
	ray.d2.data.rD = float3(1.0f / ray.d1.data.D.x, 1.0f / ray.d1.data.D.y, 1.0f / ray.d1.data.D.z);

	// use local stack instead of recursion
	TLASNode *node = &tlas_node[0], *stack[64];
	uint stack_ptr = 0;
	while (1) {
		if (node->is_leaf()) {
			blas[node->d1.data.blas].intersect(ray, node->d1.data.blas);
			if (stack_ptr == 0)
				break;
			else
				node = stack[--stack_ptr];
			continue;
		}
		TLASNode *child1 = &tlas_node[node->d0.data1.left_right & 0xffff];
		TLASNode *child2 = &tlas_node[node->d0.data1.left_right >> 16];

		float dist1 = IntersectAABB(ray, child1->d0.aabbMin, child1->d1.aabbMax);
		float dist2 = IntersectAABB(ray, child2->d0.aabbMin, child2->d1.aabbMax);

		if (dist1 > dist2) {
			std::swap(child1, child2);
			std::swap(dist1, dist2);
		}
		if (dist1 == 1e30f) {
			if (stack_ptr == 0)
				break;
			else
				node = stack[--stack_ptr];
		} else {
			node = child1;
			if (dist2 != 1e30f)
				stack[stack_ptr++] = child2;
		}
	}
}
} // namespace mujoco_ros::contact_surfaces::sensors

using namespace mujoco_ros::contact_surfaces::sensors;

int main(int argc, char **argv)
{
	auto tmp = GenerateContactSurface();
	std::shared_ptr<drake::geometry::ContactSurface<double>> contact_surface(&tmp);
	BVH bvh(contact_surface);

	// with multiple BVHs:
	/*
	BVH bvh2(contact_surface2);

	BVH bvhs[2] = {bvh, bvh2};
	tlas = TLAS(bvhs, 2);
	tlas.build();
	*/

	// bvh.build();

	std::cout << "Exiting program" << std::endl;
	return 0;
}
