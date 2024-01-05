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
/* Authors: Florian Patzelt*/

#pragma once

#include <dynamic_reconfigure/server.h>
#include <mujoco_contact_surface_sensors/DynamicFlatTactileConfig.h>

#include <mujoco_contact_surface_sensors/tactile_sensor_base.h>
#include <mujoco_contact_surface_sensors/bvh.h>

namespace mujoco_ros::contact_surfaces::sensors {
using namespace mujoco_ros::contact_surfaces;

const static float SQRT_2 = 1.41421356237;

class FlatTactileSensor : public TactileSensorBase
{
public:
	// Overloaded entry point
	virtual bool load(const mjModel *m, mjData *d) override;

protected:
	virtual void internal_update(const mjModel *m, mjData *d,
	                             const std::vector<GeomCollisionPtr> &geomCollisions) override;
	void bvh_update(const mjModel *m, mjData *d, const std::vector<GeomCollisionPtr> &geomCollisions);

	/**
	 * @brief Renders the tactile sensor tiles in the mujoco scene.
	 * @param[in] pressure The pressure values to render.
	 * @param[in] rot The rotation of the sensor.
	 * @param[in] xpos The centroid position of the sensor in global coordinates.
	 * @param[in] topleft The top left corner of the sensor in local coordinates.
	 */
	void render_tiles(Eigen::ArrayXXf pressure, mjtNum rot[9], mjtNum xpos[3], mjtNum topleft[3]);

private:
	bool use_parallel       = true;
	bool use_gaussian       = false;
	bool use_tukey          = false;
	bool use_square         = false;
	float sigma             = -1.0;
	int sampling_resolution = 5; // 25 samples per cell
	double resolution;
	int cx, cy;
	// color scaling factors for tactile visualization
	float tactile_running_scale = 3.;
	float tactile_current_scale = 0.;
	Eigen::Vector3d sensor_normal;

	float max_dist;
	float di_factor;
	float sub_halfwidth;
	float rmean;
	float rSampling_resolution;

	// Dynamic reconfigure
	dynamic_reconfigure::Server<mujoco_contact_surface_sensors::DynamicFlatTactileConfig> dynamic_param_server;
	void dynamicParamCallback(mujoco_contact_surface_sensors::DynamicFlatTactileConfig &config, uint32_t level,
	                          const mjModel *m);
	std::mutex dynamic_param_mutex;
};

} // namespace mujoco_ros::contact_surfaces::sensors
