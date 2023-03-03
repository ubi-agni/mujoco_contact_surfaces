/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University
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

/* Authors: Florian Patzelt*/

#pragma once

#include <drake/geometry/query_results/contact_surface.h>
#include <mujoco_ros/common_types.h>

namespace mujoco_contact_surfaces {

// Maximum number of geoms for visualization
const int MAX_VGEOM = 10000;

// Point collision that approximates a collision area
typedef struct PointCollision
{
	drake::Vector3<double> p;
	drake::Vector3<double> n;
	double fn0;
	double stiffness;
	double damping;
	int face;
} PointCollision;

// Collision between two geoms
typedef struct GeomCollision
{
	std::vector<PointCollision> pointCollisions;
	std::shared_ptr<drake::geometry::ContactSurface<double>> s;
	int g1;
	int g2;
	GeomCollision(int g1, int g2, drake::geometry::ContactSurface<double> *s) : g1(g1), g2(g2), s(s){};
} GeomCollision;

// SurfacePlugin
class SurfacePlugin;

/**
 * @def SurfacePluginPtr
 * @brief boost::shared_ptr to SurfacePlugin
 */
typedef boost::shared_ptr<SurfacePlugin> SurfacePluginPtr;
typedef boost::shared_ptr<GeomCollision> GeomCollisionPtr;

} // namespace mujoco_contact_surfaces