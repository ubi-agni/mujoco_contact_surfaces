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
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE /*********************************************************************
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
/* Authors: Florian Patzelt*/

#pragma once

#include <mujoco_contact_surface_sensors/tactile_sensor_base.h>
#include <mujoco_contact_surface_sensors/bvh.h>
#include <std_srvs/Empty.h>

namespace mujoco_ros::contact_surfaces::sensors {
using namespace drake;
using namespace mujoco_ros::contact_surfaces;
using namespace std_srvs;

enum TaxelMethod
{
	// For each taxel find the closest point sampled on the contact surface and compute pressure value at that point
	CLOSEST,
	WEIGHTED,
	MEAN,
	SQUARED
};

enum SampleMethod
{
	DEFAULT,
	AREA_IMPORTANCE
};

class CurvedSensor : public TactileSensorBase
{
public:
	// Overlead entry point
	virtual bool load(const mjModel * m, mjData * d);

protected:
	virtual void internal_update(const mjModel *m, mjData *d, const std::vector<GeomCollisionPtr> &geomCollisions);

private:
	// Taxel positions relative to the sensor geom pose	
	std::vector<std::vector<int>> surface_idx;
	std::vector<std::vector<double>> surface_weight;
	Eigen::Matrix<double, 4, Eigen::Dynamic> taxel_point_mat;
	Eigen::Matrix<double, 4, Eigen::Dynamic> taxel_normal_mat;
	Eigen::Matrix<double, 4, Eigen::Dynamic> surface_point_mat;
	Eigen::Matrix<double, 4, Eigen::Dynamic> surface_normal_mat;
	// Distance margin: Determines how far away surface points can be to a taxel position to still be included in the
	// sensor value computation
	double include_margin, include_margin_sq;
	// Resolution on how dense points are sampled on the contact surface
	double sample_resolution;
	// Method to compute sensor values
	TaxelMethod method;
	SampleMethod sample_method;
	// color scaling factors for tactile visualization
	double tactile_running_scale = 3.;
	double tactile_current_scale = 0.;

	double max_pressure = 0.04;

	int visualization_mode = 0;
	ros::ServiceServer toogle_service;
	void sample_triangle(int n, Vector3<double> A, Vector3<double> B, Vector3<double> C, double area, std::vector<Vector3<double>> &samples, std::vector<double> &areas);
	bool toogle_vis(EmptyRequest  &req, EmptyResponse &res);
};

} // namespace mujoco_ros::contact_surfaces::sensors
