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

#include <mujoco_contact_surface_sensors/taxel_sensor.h>

#include <pluginlib/class_list_macros.h>

namespace mujoco_contact_surface_sensors {
using namespace drake;
using namespace drake::geometry;

bool TaxelSensor::load(mjModelPtr m, mjDataPtr d)
{
	if (TactileSensorBase::load(m, d) && rosparam_config_.hasMember("taxels") && rosparam_config_.hasMember("method") &&
	    rosparam_config_.hasMember("include_margin") && rosparam_config_.hasMember("sample_resolution")) {
		include_margin                  = static_cast<double>(rosparam_config_["include_margin"]);
		sample_resolution               = static_cast<double>(rosparam_config_["sample_resolution"]);
		const std::string method_string = static_cast<std::string>(rosparam_config_["method"]);
		if (method_string == "closest") {
			method = CLOSEST;
		} else if (method_string == "weighted") {
			method = WEIGHTED;
		} else {
			ROS_ERROR_STREAM_NAMED("mujoco_contact_surface_sensors",
			                       "Could not find any match for method: " << method_string);
			return false;
		}
		auto taxel_array = rosparam_config_["taxels"];
		if (taxel_array.getType() == XmlRpc::XmlRpcValue::TypeArray && taxel_array.size() > 0) {
			int n = taxel_array.size();
			if (visualize) {
				vGeoms = new mjvGeom[mujoco_contact_surfaces::MAX_VGEOM];
			}
			taxel_mat = Eigen::Matrix<double, 4, Eigen::Dynamic>(4, n);

			for (int i = 0; i < n; ++i) {
				if (taxel_array[i].getType() == XmlRpc::XmlRpcValue::TypeArray && taxel_array[i].size() == 3) {
					Vector3<double> t(static_cast<double>(taxel_array[i][0]), static_cast<double>(taxel_array[i][1]),
					                  static_cast<double>(taxel_array[i][2]));
					taxels.push_back(t);
					taxel_mat.col(i) << t[0], t[1], t[2], 1;
				} else {
					return false;
				}
			}
		} else {
			return false;
		}

		ROS_INFO_STREAM_NAMED("mujoco_contact_surface_sensors",
		                      "Found taxel sensor '" << sensorName << "' with " << taxels.size() << " taxels.");
		sensor_msgs::ChannelFloat32 channel;
		channel.values.resize(taxels.size());
		channel.name = sensorName;
		tactile_state_msg_.sensors.push_back(channel);
		return true;
	}
	return false;
}

void TaxelSensor::internal_update(const mjModel *model, mjData *data,
                                  const std::vector<GeomCollision *> &geomCollisions)
{
	if (visualize) {
		// reset the visualized geoms
		tactile_running_scale = 0.9 * tactile_running_scale + 0.1 * tactile_current_scale;
		tactile_current_scale = 0.;
	}
	int id = geomID;

	// prepare caches
	std::vector<int> ts;
	std::vector<Vector3<double>> barys, spoints;
	std::vector<std::shared_ptr<ContactSurface<double>>> surfaces;

	for (GeomCollision *gc : geomCollisions) {
		if (gc->g1 == id or gc->g2 == id) {
			std::shared_ptr<ContactSurface<double>> s = gc->s;
			auto mesh                                 = s->tri_mesh_W();

			const int n_tri = mesh.num_elements();

			for (int t = 0; t < n_tri; ++t) {
				std::vector<Eigen::Vector3d> vertices;
				auto element = mesh.element(t);
				for (int i = 0; i < element.num_vertices(); ++i) {
					int v                     = element.vertex(i);
					const Vector3<double> &vp = mesh.vertex(v);
					vertices.push_back(vp);
				}
				// Sample points on the triangle
				int st0 = (int)((vertices[1] - vertices[0]).norm() / sample_resolution) + 1;
				int st1 = (int)((vertices[2] - vertices[0]).norm() / sample_resolution) + 1;
				int st2 = (int)((vertices[2] - vertices[0]).norm() / sample_resolution) + 1;
				int st  = std::max(st0, st1);
				for (double a = 0; a <= 1; a += 1. / st) {
					for (double b = 0; b <= 1; b += 1. / st2) {
						const Vector3<double> bary(a, (1 - a) * (1 - b), (1 - a) * b);
						Eigen::Vector3d p = bary[0] * vertices[0] + bary[1] * vertices[1] + bary[2] * vertices[2];
						// Cache sampled points on the contact surface
						barys.push_back(bary);
						spoints.push_back(p);
						surfaces.push_back(s);
						ts.push_back(t);
					}
				}
			}
		}
	}
	int m = spoints.size();
	int n = taxels.size();

	const float red[4]  = { 1, 0, 0, 1 };
	const float blue[4] = { 0, 0, 1, 1 };
	mjtNum size[3]      = { 0.001, 0.001, 0.01 };
	mjtNum sizeS[3]     = { 0.001, 0.001, 0.001 };
	mjtNum pos[3], posS[3];
	mjtNum rot[9];

	if (m > 0) {
		// Compute taxel positions at the current sensor geom position
		Eigen::Matrix4d M;
		M << data->geom_xmat[9 * id + 0], data->geom_xmat[9 * id + 1], data->geom_xmat[9 * id + 2],
		    data->geom_xpos[3 * id], data->geom_xmat[9 * id + 3], data->geom_xmat[9 * id + 4],
		    data->geom_xmat[9 * id + 5], data->geom_xpos[3 * id + 1], data->geom_xmat[9 * id + 6],
		    data->geom_xmat[9 * id + 7], data->geom_xmat[9 * id + 8], data->geom_xpos[3 * id + 2], 0, 0, 0, 1;
		Eigen::Matrix<double, 4, Eigen::Dynamic> taxels_at_M  = M * taxel_mat;
		Eigen::Matrix<double, 3, Eigen::Dynamic> taxels_at_M3 = taxels_at_M.topRows<3>();

		// Convert sampled points from the surface to Eigen Matrix
		Eigen::Matrix<double, 3, Eigen::Dynamic> surface_points(3, m);
		for (int i = 0; i < m; ++i) {
			surface_points.col(i) << spoints[i];
			for (int h = 0; h < 3; ++h) {
				posS[h] = surface_points(h, i);
			}
		}
		// Compute distance matrix between taxel positions and surface points
		Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dist(n, m);
		dist = ((-2 * taxels_at_M3.transpose() * surface_points).colwise() +
		        taxels_at_M3.colwise().squaredNorm().transpose())
		           .rowwise() +
		       surface_points.colwise().squaredNorm();

		switch (method) {
			case CLOSEST:

				for (int i = 0; i < n; ++i) {
					int j;
					// for each taxel find index of closest surface point
					double distance                           = dist.row(i).minCoeff(&j);
					std::shared_ptr<ContactSurface<double>> s = surfaces[j];
					int t                                     = ts[j];
					Vector3<double> bary                      = barys[j];

					if (visualize) {
						Vector3<double> normal = s->face_normal(t);

						for (int h = 0; h < 3; ++h) {
							pos[h]     = taxels_at_M3(h, i);
							posS[h]    = surface_points(h, j);
							rot[h + 6] = normal[h];
							rot[h + 3] = 0; // s->centroid(t)[h] - pos[h];
						}
						// mju_normalize3(rot + 3);
						if (normal[0] > 0.95) {
							rot[4] = 1;
						} else {
							rot[3] = 1;
						}
						mju_cross(rot, rot + 3, rot + 6);
						initVGeom(mjGEOM_SPHERE, sizeS, pos, NULL, red);
					}

					// If computed closest surface point is in margin, compute pressure at that point
					if (distance < include_margin) {
						double pressure                         = s->tri_e_MN().Evaluate(t, bary) * s->area(t);
						tactile_state_msg_.sensors[0].values[i] = pressure;
						if (visualize && std::abs(pressure) > 1e-6) {
							tactile_current_scale = std::max(std::abs(pressure), tactile_current_scale);
							size[2] = std::min(std::abs(pressure), tactile_running_scale) / tactile_running_scale / 50.;
							initVGeom(mjGEOM_ARROW, size, pos, rot, red);
							initVGeom(mjGEOM_SPHERE, sizeS, posS, NULL, blue);
						}

					} else {
						tactile_state_msg_.sensors[0].values[i] = 0;
					}
				}
				break;
			case WEIGHTED:
				for (int i = 0; i < n; ++i) {
					double ws       = 0;
					double pressure = 0;
					Vector3<double> normal(0, 0, 0);

					if (visualize) {
						for (int h = 0; h < 3; ++h) {
							pos[h] = taxels_at_M3(h, i);
						}
						initVGeom(mjGEOM_SPHERE, sizeS, pos, NULL, red);
					}

					for (int j = 0; j < m; ++j) {
						if (dist(i, j) < include_margin) {
							std::shared_ptr<ContactSurface<double>> s = surfaces[j];
							int t                                     = ts[j];
							Vector3<double> bary                      = barys[j];
							double w                                  = include_margin - dist(i, j);
							double p = w * std::abs(s->tri_e_MN().Evaluate(t, bary) * s->area(t));
							pressure += p;
							ws += w;
							if (visualize) {
								normal += p * s->face_normal(t);
								for (int h = 0; h < 3; ++h) {
									posS[h] = surface_points(h, j);
								}
								initVGeom(mjGEOM_SPHERE, sizeS, posS, NULL, blue);
							}
						}
					}
					if (ws > 0) {
						// normal /= pressure;
						pressure /= ws;
						tactile_state_msg_.sensors[0].values[i] = pressure;
						if (visualize) {
							tactile_current_scale = std::max(std::abs(pressure), tactile_current_scale);
							size[2] = std::min(std::abs(pressure), tactile_running_scale) / tactile_running_scale / 50.;
							for (int h = 0; h < 3; ++h) {
								pos[h]     = taxels_at_M3(h, i);
								rot[h + 6] = normal[h];
								rot[h + 3] = 0;
							}
							if (normal[0] > 0.95) {
								rot[4] = 1;
							} else {
								rot[3] = 1;
							}
							mju_normalize3(rot + 6);
							mju_cross(rot, rot + 3, rot + 6);
							initVGeom(mjGEOM_ARROW, size, pos, rot, red);
						}
					}
				}
		}
	} else {
		// If there are no sampled points on the surface fill the sensor message with zeros
		for (int i = 0; i < n; ++i) {
			tactile_state_msg_.sensors[0].values[i] = 0;
		}
	}
}

} // namespace mujoco_contact_surface_sensors

PLUGINLIB_EXPORT_CLASS(mujoco_contact_surface_sensors::TaxelSensor, mujoco_contact_surfaces::SurfacePlugin)
