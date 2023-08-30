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

#include <mujoco_contact_surface_sensors/flat_tactile_sensor.h>
#include <typeinfo>
#include <cmath>
#include <omp.h>

#include <pluginlib/class_list_macros.h>

namespace mujoco_ros::contact_surfaces::sensors {
using namespace drake;
using namespace drake::geometry;

void FlatTactileSensor::dynamicParamCallback(mujoco_contact_surface_sensors::DynamicFlatTactileConfig &config,
                                             uint32_t level, mjModelPtr m)
{
	std::lock_guard<std::mutex> lock(dynamic_param_mutex);
	ROS_INFO_STREAM("Reconfigure request for " << sensorName << " received.");

	updatePeriod        = ros::Duration(1.0 / config.update_rate);
	visualize           = config.visualize;
	resolution          = config.resolution;
	sampling_resolution = config.sampling_resolution;
	use_parallel        = config.use_parallel;
	sigma               = static_cast<float>(config.sigma);

	//// precompute some factors for the update loop to save time

	// distance between two adjacent samples on the same axis:
	// dist_factor = resolution/sampling_resolution
	// distance between left edge of sensor and sample i:
	// edgedist_i = dist_factor * i + 0.5 * dist_factor
	// Taking the absolute distance between di and half the cell width gives the distance to the center of the cell:
	// edgedist_i - resolution/2 <=> dist_factor * i + 0.5 * dist_factor - resolution/2 <=> dist_factor * i +
	// sub_halfwidth
	di_factor     = resolution / sampling_resolution;
	sub_halfwidth = di_factor / 2.0f - resolution / 2.0f;

	rmean                = 1. / (sampling_resolution * sampling_resolution);
	rSampling_resolution = resolution / sampling_resolution;

	max_dist = SQRT_2 * resolution / 2.0f;

	if (config.window == 1) {
		use_gaussian = true;
		use_tukey    = false;
		use_square   = false;
		ROS_INFO_STREAM("\tUsing gaussian window with sigma = " << sigma);
	} else if (config.window == 2) {
		use_gaussian = false;
		use_tukey    = true;
		use_square   = false;
		ROS_INFO_STREAM("\tUsing tukey window with sigma (alpha) = " << sigma);
	} else if (config.window == 3) {
		use_gaussian = false;
		use_tukey    = false;
		use_square   = true;
		ROS_INFO_STREAM("\tUsing squared distance window");
	} else {
		use_gaussian = false;
		use_tukey    = false;
		use_square   = false;
		ROS_INFO_STREAM("\tUsing no window");
	}

	double xs = m->geom_size[3 * geomID];
	double ys = m->geom_size[3 * geomID + 1];
	// std::floorl gives compiler errors, this is a known bug:
	// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=79700
	cx     = ::floorl(2 * xs / resolution + 0.1); // add 0.1 to counter wrong flooring due to imprecision
	cy     = ::floorl(2 * ys / resolution + 0.1); // add 0.1 to counter wrong flooring due to imprecision
	vGeoms = new mjvGeom[2 * cx * cy + 50];
	ROS_INFO_STREAM("\tResolution set to " << cx << "x" << cy);

	tactile_state_msg_.sensors.clear();

	sensor_msgs::ChannelFloat32 channel;
	channel.values.resize(cx * cy);
	channel.name = sensorName;
	tactile_state_msg_.sensors.push_back(channel);
}

bool FlatTactileSensor::load(mjModelPtr m, mjDataPtr d)
{
	if (TactileSensorBase::load(m, d) && rosparam_config_.hasMember("resolution")) {
		resolution = static_cast<double>(rosparam_config_["resolution"]);

		if (rosparam_config_.hasMember("sampling_resolution")) {
			sampling_resolution = static_cast<int>(rosparam_config_["sampling_resolution"]);
		}

		if (rosparam_config_.hasMember("use_parallel")) {
			use_parallel = static_cast<bool>(rosparam_config_["use_parallel"]);
		}

		if (rosparam_config_.hasMember("windowing")) {
			std::string windowing = static_cast<std::string>(rosparam_config_["windowing"]);

			if (rosparam_config_.hasMember("sigma")) {
				// direct cast to float is ambiguous, so we need to cast to double first
				sigma = static_cast<float>(static_cast<double>(rosparam_config_["sigma"]));
			}

			if (windowing == "gauss") {
				use_gaussian = true;
				if (sigma == -1.0) {
					ROS_WARN_STREAM("No sigma given for gaussian window. Falling back to default (sigma = 0.1).");
					sigma = 0.1;
				}
				ROS_DEBUG_STREAM("Using gaussian window with sigma = " << sigma);
			} else if (windowing == "tukey") {
				use_tukey = true;
				if (sigma == -1.0) {
					ROS_WARN_STREAM("No sigma given for tukey window. Falling back to default (sigma = 0.3).");
					sigma = 0.3;
				}
				ROS_DEBUG_STREAM("Using tukey window with sigma (alpha) = " << sigma);
			} else if (windowing == "square") {
				use_square = true;
				ROS_DEBUG_STREAM("Using squared distance window.");
			} else {
				ROS_WARN_STREAM("Unknown windowing function: " << windowing << ". Falling back to default (none).");
			}
		}

		//// precompute some factors for the update loop to save time

		// distance between two adjacent samples on the same axis:
		// dist_factor = resolution/sampling_resolution
		// distance between left edge of sensor and sample i:
		// edgedist_i = dist_factor * i + 0.5 * dist_factor
		// Taking the absolute distance between di and half the cell width gives the distance to the center of the cell:
		// edgedist_i - resolution/2 <=> dist_factor * i + 0.5 * dist_factor - resolution/2 <=> dist_factor * i +
		// sub_halfwidth
		di_factor     = resolution / sampling_resolution;
		sub_halfwidth = di_factor / 2.0f - resolution / 2.0f;

		rmean                = 1. / (sampling_resolution * sampling_resolution);
		rSampling_resolution = resolution / sampling_resolution;

		max_dist = SQRT_2 * resolution / 2.0f;

		// TODO: Other namespace?
		// dynamic_param_server(ros::NodeHandle(node_handle_->getNamespace() + "/" + sensorName));
		dynamic_param_server.setCallback(boost::bind(&FlatTactileSensor::dynamicParamCallback, this, _1, _2, m));

		double xs = m->geom_size[3 * geomID];
		double ys = m->geom_size[3 * geomID + 1];
		// std::floorl gives compiler errors, this is a known bug:
		// https://gcc.gnu.org/bugzilla/show_bug.cgi?id=79700
		cx     = ::floorl(2 * xs / resolution + 0.1); // add 0.1 to counter wrong flooring due to imprecision
		cy     = ::floorl(2 * ys / resolution + 0.1); // add 0.1 to counter wrong flooring due to imprecision
		vGeoms = new mjvGeom[2 * cx * cy + 50];
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

void FlatTactileSensor::internal_update(const mjModel *m, mjData *d,
                                        const std::vector<GeomCollisionPtr> &geomCollisions)
{
	std::lock_guard<std::mutex> lock(dynamic_param_mutex);
	bvh_update(m, d, geomCollisions);
}

void FlatTactileSensor::render_tiles(Eigen::ArrayXXf pressure, mjtNum rot[9], mjtNum xpos[3], mjtNum topleft[3])
{
	tactile_running_scale = 0.9f * tactile_running_scale + 0.1f * tactile_current_scale;

	if (!visualize)
		return;

	mjtNum size[3]        = { resolution / 2.f, resolution / 2.f, 0.0005 };
	mjtNum sphere_size[3] = { 0.0005, 0.0005, 0.0005 };
	const float rgbaA[4]  = { 1.f, 0.0f, 0.f, 0.3f };
	const float rgbaC[4]  = { 1.0f, 1.0f, 1.f, 0.8f };

	for (int x = 0; x < cx; x++) {
		for (int y = 0; y < cy; y++) {
			double mean_pressure = pressure(x, y);

			float ps            = std::fmin(std::fabs(mean_pressure), tactile_running_scale) / tactile_running_scale;
			const float rgba[4] = { ps, 0, (1.f - ps), 0.8 };

			mjtNum pos[3] = { topleft[0] + x * resolution + (resolution / 2),
				               topleft[1] + y * resolution + (resolution / 2), topleft[2] };
			mju_rotVecMat(pos, pos, rot);
			mju_addTo3(pos, xpos);

			mjvGeom *g = vGeoms + n_vGeom++;
			mjv_initGeom(g, mjGEOM_BOX, size, pos, rot, rgba);

			// Points and arrow vis
			// g = vGeoms + n_vGeom++;
			// mjv_initGeom(g, mjGEOM_SPHERE, sphere_size, pos, rot, rgbaC);
			if (mean_pressure > 0.0) {
				g               = vGeoms + n_vGeom++;
				mjtNum sizeA[3] = { 0.001, 0.001, 0.1 * ps };
				mjv_initGeom(g, mjGEOM_ARROW, sizeA, pos, rot, rgbaA);
			}
		}
	}
}

void FlatTactileSensor::bvh_update(const mjModel *m, mjData *d, const std::vector<GeomCollisionPtr> &geomCollisions)
{
	int id    = geomID;
	float xs  = static_cast<float>(m->geom_size[3 * id]);
	float ys  = static_cast<float>(m->geom_size[3 * id + 1]);
	float zs  = static_cast<float>(m->geom_size[3 * id + 2]);
	float res = static_cast<float>(resolution);

	tactile_current_scale = 0.f;

	mjtNum rot[9];
	mju_copy(rot, d->geom_xmat + 9 * id, 9);

	// Negative Z axis is defined as the normal of the flat sensor (sensor points are outside of the object and point
	// inward)
	float3 sensor_normal(static_cast<float>(-rot[2]), static_cast<float>(-rot[5]), static_cast<float>(-rot[8]));

	mjtNum sensor_xpos[3]    = { d->geom_xpos[3 * geomID], d->geom_xpos[3 * geomID + 1], d->geom_xpos[3 * geomID + 2] };
	mjtNum sensor_topleft[3] = { -xs, -ys, zs };

	BVH bvh[geomCollisions.size()];
	int bvh_idx = 0;

	Eigen::ArrayXXf pressure = Eigen::ArrayXXf::Zero(cx, cy);
	for (GeomCollisionPtr gc : geomCollisions) {
		if (gc->g1 == id or gc->g2 == id) {
			bvh[bvh_idx] = BVH(gc->s);
			bvh_idx++;
		}
	}
	if (bvh_idx == 0) {
		tactile_state_msg_.sensors[0].values.clear();
		tactile_state_msg_.sensors[0].values.resize(cx * cy);
		memcpy(std::data(tactile_state_msg_.sensors[0].values), pressure.data(), cx * cy * sizeof(float));
		render_tiles(pressure, rot, sensor_xpos, sensor_topleft);
		return; // no contacts with surfaces in this timestep
	}

	TLAS tlas(bvh, bvh_idx);
	tlas.build();

	/////// Precompute as much of the factors needed in the loop as possible (especially divisions) to save time
	float *pressure_raw  = pressure.data();
	float cell_halfwidth = resolution / 2.0f;

	// Gaussian weighting
	float rsigma_squared;
	float factor = 1.f;
	float rtukey = 0.f;
	if (use_gaussian) {
		rsigma_squared = 0.5f / (sigma * sigma);
		factor         = 1.f / (sigma * sqrt(2.f * M_PI));
	} else if (use_tukey) {
		rtukey = 1.f / sigma * sampling_resolution * sampling_resolution;
	}

	ROS_DEBUG_STREAM_ONCE("rmean: " << rmean);
	for (int x = 0; x < cx; x++) { // for each cell on axis x
		for (int y = 0; y < cy; y++) { // for each cell on axis y
			float avg_pressure = 0;
			{
// #pragma omp declare reduction (+: Eigen::ArrayXXd : omp_out += omp_in) initializer(omp_priv=Eigen::ArrayXXd::Zero(cx,
// cy)) #pragma omp target map(tofrom : pressure_raw) map(to : x, y, t, i, j, sensor_center, rSampling_resolution, xs,
// ys, zs, tlas) if (use_parallel)
#pragma omp parallel for reduction(+ : avg_pressure) schedule(dynamic, 8) if (use_parallel && sampling_resolution > 8)
				for (int i = 0; i < sampling_resolution; i++) { // for each sample in cell on axis x
					for (int j = 0; j < sampling_resolution; j++) { // for each sample in cell on axis y

						mjtNum pos[3] = {
							sensor_topleft[0] + x * resolution + i * rSampling_resolution + 0.5 * rSampling_resolution,
							sensor_topleft[1] + y * resolution + j * rSampling_resolution + 0.5 * rSampling_resolution,
							1.5 * zs
						};
						mju_rotVecMat(pos, pos, rot);
						mju_addTo3(pos, sensor_xpos);

						float3 sensor_point = float3(pos[0], pos[1], pos[2]);

						Ray ray;
						ray.d0.data.O = sensor_point;
						ray.d1.data.D = sensor_normal;
						ray.hit.t     = 1e30f;

						tlas.intersect(ray);
						// Instead of using 1e30f, we use the maximum distance to the sensor geom z-axis centroid (local
						// frame) to prevent contacts from the wrong side of the sensor to be considered.
						// TODO(dleins): Maybe we should move the maximum distance as a parameter to the ray intersection
						// functions, to directly discard AABBs that are too far away and avoid more intersection tests
						if (ray.hit.t < 1.5 * zs && ray.hit.t > 0.0f) {
							const Eigen::Vector3d bary(1 - ray.hit.u - ray.hit.v, ray.hit.u, ray.hit.v);
							uint tri_idx  = ray.hit.bvh_triangle & 0xFFFFF;
							uint blas_idx = ray.hit.bvh_triangle >> 20;
							float raw     = tlas.blas[blas_idx].surface->tri_e_MN().Evaluate(tri_idx, bary) * rmean;
							float weight  = 1.0f;

							// Gaussian kernel
							// g(x) = 1/(sigma*sqrt(2*pi)) * exp(-0.5*pow(x - mu, 2)/pow(sigma, 2))
							if (use_gaussian) {
								// Compute the 2D-distance between the sensor center and the ray
								float dist =
								    std::hypot(di_factor * i + sub_halfwidth, di_factor * j + sub_halfwidth) / max_dist;
								// Compute the weight of the sample using the Gaussian function
								weight = exp(-(dist * dist) * rsigma_squared);
							}

							// Tukey kernel (alpha = 1 -> hann window, alpha = 0 -> rectangular window)
							// w[n] = 1/2 (1 - cos(2*pi*n/(alpha*N))), if 0 <= n <= alpha*N/2
							// w[n] = 1, if alpha*N/2 < n <= (N/2)
							// w[N-n] = w[n], if 0 <= n <= N/2
							else if (use_tukey) {
								if (sampling_resolution / 2 - abs(sampling_resolution / 2 - i) <=
								    sigma * sampling_resolution / 2) {
									weight *= 0.5f * (1.f - cosf(2.f * M_PI *
									                             (sampling_resolution / 2 - abs(sampling_resolution / 2 - i)) *
									                             rtukey));
								}
								if (sampling_resolution / 2 - abs(sampling_resolution / 2 - j) <=
								    sigma * sampling_resolution / 2) {
									weight *= 0.5f * (1.f - cosf(2.f * M_PI *
									                             (sampling_resolution / 2 - abs(sampling_resolution / 2 - j)) *
									                             rtukey));
								}
								// else weight remains 1
							}

							else if (use_square) {
								// Compute the inverse of the 2D-distance between the sensor center and the ray
								float inv_dist =
								    1 - std::hypot(di_factor * i + sub_halfwidth, di_factor * j + sub_halfwidth) / max_dist;
								weight = inv_dist * inv_dist;
							}

							// Add the weighted sample to the average pressure
							avg_pressure += weight * raw;
						}
					}
				}
			}
			pressure_raw[x + cy * y]                         = avg_pressure;
			tactile_state_msg_.sensors[0].values[x + cy * y] = avg_pressure;
			tactile_current_scale                            = std::fmax(std::fabs(avg_pressure), tactile_current_scale);
		}
	}
	render_tiles(pressure, rot, sensor_xpos, sensor_topleft);
}

} // namespace mujoco_ros::contact_surfaces::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::contact_surfaces::sensors::FlatTactileSensor,
                       mujoco_ros::contact_surfaces::SurfacePlugin)
