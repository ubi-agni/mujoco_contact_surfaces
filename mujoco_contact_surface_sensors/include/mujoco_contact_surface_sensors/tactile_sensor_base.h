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

#include <mujoco_contact_surfaces/plugin_utils.h>
#include <tactile_msgs/TactileState.h>

namespace mujoco_contact_surface_sensors {
using namespace mujoco_contact_surfaces;

using namespace std::chrono;
using namespace MujocoSim;

class TactileSensorBase : public SurfacePlugin
{
public:
	// Overlead entry point
	virtual bool load(mjModelPtr m, mjDataPtr d);
	virtual void update(const mjModel *m, mjData *d, const std::vector<GeomCollision *> &geomCollisions);
	virtual void renderCallback(mjModelPtr model, mjDataPtr data, mjvScene *scene);
	virtual void reset();

private:
	// Buffer of visual geoms
	// color scaling factors for tactile visualization
	double tactile_running_scale = 3.;
	double tactile_current_scale = 0.;

protected:
	// MuJoCo id of the geom this sensor is attached to
	int geomID;
    // Name of the geom this sensor is attached to
    std::string geomName;

	// update frequency of the sensor
	double updateRate;
	ros::Duration updatePeriod;
	// time of last sensor update
	ros::Time lastUpdate;

	// ros publisher for sensor data
	ros::Publisher publisher;
    tactile_msgs::TactileState tactile_state_msg_;
	std::string topicName;
    std::string sensorName;

	bool visualize = false;
	// geom buffer used for visualization
	mjvGeom *vGeoms;
	// number of geoms in vGeoms
	int n_vGeom;

	virtual void internal_update(const mjModel *m, mjData *d, const std::vector<GeomCollision *> &geomCollisions){};

private:
};

} // namespace mujoco_contact_surface_sensors
