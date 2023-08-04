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

#include <mujoco_contact_surface_sensors/flat_tactile_sensor.h>

#include <pluginlib/class_list_macros.h>

namespace mujoco_contact_surface_sensors {
using namespace drake;
using namespace drake::geometry;

bool FlatTactileSensor::load(mjModelPtr m, mjDataPtr d)
{
	if (TactileSensorBase::load(m, d) && rosparam_config_.hasMember("resolution")) {
		resolution = static_cast<double>(rosparam_config_["resolution"]);

		if (rosparam_config_.hasMember("sampling_resolution")) {
			sampling_resolution = static_cast<int>(rosparam_config_["sampling_resolution"]);
		} else {
			ROS_ERROR("Sampling resolution not provided by config file!");
			return false;
		}

		double xs = m->geom_size[3 * geomID];
		double ys = m->geom_size[3 * geomID + 1];
		// std::floorl gives compiler errors, this is a known bug:
		// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=79700
		cx     = ::floorl(2 * xs / resolution + 0.1); // add 0.1 to counter wrong flooring due to imprecision
		cy     = ::floorl(2 * ys / resolution + 0.1); // add 0.1 to counter wrong flooring due to imprecision
		vGeoms = new mjvGeom[2 * cx * cy + 50]; // reserve 50 vGeoms more for testing
		ROS_INFO_STREAM_NAMED("mujoco_contact_surface_sensors",
		                      "Found tactile sensor: " << sensorName << " " << cx << "x" << cy);
		sensor_msgs::ChannelFloat32 channel;
		channel.values.resize(cx * cy);
		channel.name = sensorName;
		tactile_state_msg_.sensors.push_back(channel);
		return true;
	}
	return false;
}

void FlatTactileSensor::render_tiles(Eigen::ArrayXXf pressure, mjtNum rot[9], mjtNum origin[3])
{
	if (!visualize)
		return;

	const mjtNum size[3]        = { resolution / 2.f, resolution / 2.f, 0.0005f };
	const mjtNum sphere_size[3] = { 0.0005f, 0.0005f, 0.0005f };
	const float rgbaA[4]        = { 1.f, 0.0f, 0.f, 0.3f };
	const float rgbaC[4]        = { 1.f, 1.0f, 1.f, 0.3f };

	for (int x = 0; x < cx; x++) {
		for (int y = 0; y < cy; y++) {
			double mean_pressure = pressure(x, y);

			tactile_current_scale = std::max(std::abs(mean_pressure), tactile_current_scale);
			float ps              = std::min(std::abs(mean_pressure), tactile_current_scale) / tactile_running_scale;
			const float rgba[4]   = { ps, 0, (1.f - ps), 0.8 };

			mjtNum pos[3] = { origin[0] + x * resolution + (resolution / 2), origin[1] + y * resolution + (resolution / 2),
				               origin[2] };

			mjvGeom *g = vGeoms + n_vGeom++;
			mjv_initGeom(g, mjGEOM_BOX, size, pos, rot, rgba);

			if (mean_pressure > 0.0) {
				g               = vGeoms + n_vGeom++;
				mjtNum sizeA[3] = { 0.001, 0.001, 0.1 * ps };
				mjv_initGeom(g, mjGEOM_ARROW, sizeA, pos, rot, rgbaA);
			}
		}
	}
}

void FlatTactileSensor::internal_update(const mjModel *m, mjData *d,
                                        const std::vector<GeomCollisionPtr> &geomCollisions)
{
	if (visualize) {
		// reset the visualized geoms
		tactile_running_scale = 0.9 * tactile_running_scale + 0.1 * tactile_current_scale;
		tactile_current_scale = 0.;
	}
	int id   = geomID;
	float xs = static_cast<float>(m->geom_size[3 * id]);
	float ys = static_cast<float>(m->geom_size[3 * id + 1]);
	float zs = static_cast<float>(m->geom_size[3 * id + 2]);

	mjtNum rot[9];
	mju_copy(rot, d->geom_xmat + 9 * id, 9);

	// Negative Z axis is defined as the normal of the flat sensor
	// (sensor points are outside of the object and point inward)
	float3 sensor_normal(static_cast<float>(d->geom_xmat[9 * geomID + 2]),
	                     static_cast<float>(d->geom_xmat[9 * geomID + 5]),
	                     static_cast<float>(-d->geom_xmat[9 * geomID + 8]));

	float3 sensor_center(static_cast<float>(d->geom_xpos[3 * geomID]), static_cast<float>(d->geom_xpos[3 * geomID + 1]),
	                     static_cast<float>(d->geom_xpos[3 * geomID + 2]));

	mjtNum sensor_topleft[3] = { sensor_center[0] - xs, sensor_center[1] - ys, sensor_center[2] + zs };

	BVH bvh[geomCollisions.size()];
	int bvh_idx = 0;

	Eigen::ArrayXXd pressure = Eigen::ArrayXXd::Zero(cx, cy);

	for (GeomCollisionPtr gc : geomCollisions) {
		if (gc->g1 == id or gc->g2 == id) {
			bvh[bvh_idx] = BVH(gc->s);
			bvh_idx++;
		}
	}

	if (bvh_idx == 0) {
		render_tiles(pressure, rot, sensor_topleft);
		return; // no contacts with surfaces in this timestep
	}

	TLAS tlas(bvh, bvh_idx);
	tlas.build();

	float *pressure_raw        = pressure.data();
	float rmean                = 1.f / (sampling_resolution * sampling_resolution);
	float rSampling_resolution = resolution / sampling_resolution;
	for (int x = 0; x < cx; x++) {
		for (int y = 0; y < cy; y++) {
			double avg_pressure = 0;
			for (int i = 0; i < sampling_resolution; i++) {
				for (int j = 0; j < sampling_resolution; j++) {
					float3 sensor_point =
					    sensor_center + float3(-xs + x * resolution + i * rSampling_resolution + 0.5 * rSampling_resolution,
					                           -ys + y * resolution + j * rSampling_resolution + 0.5 * rSampling_resolution,
					                           1.5 * zs);

					Ray ray;
					ray.d0.data.O = sensor_point;
					ray.d1.data.D = sensor_normal;
					ray.hit.t     = 1e30f;

					tlas.intersect(ray);

					if (ray.hit.t < 1e30f && ray.hit.t > 0.0f) {
						const Eigen::Vector3d bary(1 - ray.hit.u - ray.hit.v, ray.hit.u, ray.hit.v);

						uint tri_idx  = ray.hit.bvh_triangle & 0xFFFFF;
						uint blas_idx = ray.hit.bvh_triangle >> 20;
						avg_pressure += tlas.blas[blas_idx].surface->tri_e_MN().Evaluate(tri_idx, bary) * rmean;
					}
				}
			}
			pressure_raw[x + cy * y]                         = avg_pressure;
			tactile_state_msg_.sensors[0].values[x + cy * y] = avg_pressure;
		}
	}
	render_tiles(pressure, rot, sensor_topleft);
}
} // namespace mujoco_contact_surface_sensors

PLUGINLIB_EXPORT_CLASS(mujoco_contact_surface_sensors::FlatTactileSensor, mujoco_contact_surfaces::SurfacePlugin)
