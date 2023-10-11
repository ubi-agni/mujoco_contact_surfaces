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

#include <mujoco_contact_surface_sensors/taxel_sensor.h>

#include <pluginlib/class_list_macros.h>
#include <random>

namespace mujoco_ros::contact_surfaces::sensors {
using namespace drake;
using namespace drake::geometry;

bool TaxelSensor::load(mjModelPtr m, mjDataPtr d)
{
	if (TactileSensorBase::load(m, d) && rosparam_config_.hasMember("taxels") && rosparam_config_.hasMember("method") &&
	    rosparam_config_.hasMember("include_margin") && rosparam_config_.hasMember("sample_resolution")) {
		include_margin    = static_cast<double>(rosparam_config_["include_margin"]);
		include_margin_sq = include_margin * include_margin;
		sample_resolution = static_cast<double>(rosparam_config_["sample_resolution"]);

		if (not rosparam_config_.hasMember("sample_method")) {
			sample_method = DEFAULT;
		} else {
			const std::string sample_method_string = static_cast<std::string>(rosparam_config_["sample_method"]);
			if (sample_method_string == "default") {
				sample_method = DEFAULT;
			} else if (sample_method_string == "area_importance") {
				sample_method = AREA_IMPORTANCE;
				ROS_INFO_STREAM_NAMED("mujoco_contact_surface_sensors", "Sample Method AREA_IMPORTANCE");
			} else {
				ROS_ERROR_STREAM_NAMED("mujoco_contact_surface_sensors",
				                       "Could not find any match for sample_method: " << sample_method_string);
				return false;
			}
		}

		if (rosparam_config_.hasMember("visualize_max_pressure")) {
			max_pressure = static_cast<double>(rosparam_config_["visualize_max_pressure"]);
		}

		const std::string method_string = static_cast<std::string>(rosparam_config_["method"]);
		if (method_string == "closest") {
			method = CLOSEST;
		} else if (method_string == "weighted") {
			method = WEIGHTED;
		} else if (method_string == "mean") {
			method = MEAN;
		} else if (method_string == "squared") {
			method = SQUARED;
		} else {
			ROS_ERROR_STREAM_NAMED("mujoco_contact_surface_sensors",
			                       "Could not find any match for method: " << method_string);
			return false;
		}
		auto taxel_array = rosparam_config_["taxels"];
		if (taxel_array.getType() == XmlRpc::XmlRpcValue::TypeArray && taxel_array.size() > 0) {
			int n = taxel_array.size();
			if (visualize) {
				vGeoms = new mjvGeom[mujoco_ros::contact_surfaces::MAX_VGEOM];
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

		if (m->geom_type[geomID] == mjGEOM_MESH) {
			mjtNum *p0 = m->geom_pos + 3 * geomID;
			mjtNum *q0 = m->geom_quat + 4 * geomID;

			if (p0[0] != 0 || p0[1] != 0 || p0[2] != 0 || q0[0] != 1 || q0[1] != 0 || q0[2] != 0 || q0[3] != 0) {
				ROS_WARN_STREAM_NAMED("mujoco_contact_surface_sensors",
				                      "Taxel sensor '"
				                          << sensorName
				                          << "' attached to a mesh geom! To ensure that the taxel poses are defined "
				                             "relative to the loaded mesh please do NOT include a transform in the <geom> "
				                             "tag. Use the transform in a wrapper <body> instead. Also do NOT define any "
				                             "refpos or refquat for the mesh.");

				// apply geom offset to taxel poses
				mjMARKSTACK;
				// int id = geomID;
				Eigen::Matrix4d M;
				mjtNum *R = mj_stackAlloc(d.get(), 9);

				mjtNum *p = mj_stackAlloc(d.get(), 3);
				mjtNum *q = mj_stackAlloc(d.get(), 4);

				mju_negPose(p, q, p0, q0);

				mju_quat2Mat(R, q);

				M << R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1;

				taxel_mat = M * taxel_mat;
				// mju_printMat(p0, 1, 3);
				// mju_printMat(q0, 1, 4);
				// std::cout << taxel_mat.topRows<3>().transpose() << std::endl;
				mjFREESTACK;
			}
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
                                  const std::vector<GeomCollisionPtr> &geomCollisions)
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

	switch (sample_method) {
		case DEFAULT:
			for (GeomCollisionPtr gc : geomCollisions) {
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
			break;
		case AREA_IMPORTANCE:
			// Class variables?
			std::default_random_engine generator;
			std::uniform_real_distribution<double> distribution(0.0, 1.0);
			for (GeomCollisionPtr gc : geomCollisions) {
				if (gc->g1 == id or gc->g2 == id) {
					std::shared_ptr<ContactSurface<double>> s = gc->s;
					auto mesh                                 = s->tri_mesh_W();

					double area_resolution = sample_resolution * mesh.total_area();
					double area            = 0;
					double at              = 0;

					const int n_tri = mesh.num_elements();

					for (int t = 0; t < n_tri; ++t) {
						auto element = mesh.element(t);
						std::vector<Eigen::Vector3d> vertices;
						at += mesh.area(t);
						while (area < at) {
							area += area_resolution;
							double u0 = distribution(generator);
							double u1 = distribution(generator);

							double a = 1.0 - sqrt(u0);
							double b = (1.0 - a) * u1;

							const Vector3<double> bary(a, (1 - a) * (1 - b), (1 - a) * b);
							Eigen::Vector3d p = bary[0] * mesh.vertex(element.vertex(0)) +
							                    bary[1] * mesh.vertex(element.vertex(1)) +
							                    bary[2] * mesh.vertex(element.vertex(2));
							// Cache sampled points on the contact surface
							barys.push_back(bary);
							spoints.push_back(p);
							surfaces.push_back(s);
							ts.push_back(t);
						}
					}
				}
			}
			break;
	}

	int m = spoints.size();
	int n = taxels.size();

	const float red[4]   = { 1, 0, 0, 1 };
	const float blue[4]  = { 0, 0, 1, 1 };
	const float green[4] = { 0, 1, 0, 1 };
	// mjtNum size[3]      = { 0.001, 0.001, 0.01 };
	mjtNum sizeS[3]  = { 0.0005, 0.0005, 0.0005 };
	mjtNum sizeSP[3] = { 0.0001, 0.0001, 0.0001 };
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
							pos[h]  = taxels_at_M3(h, i);
							posS[h] = surface_points(h, j);
						}

						initVGeom(mjGEOM_SPHERE, sizeS, pos, NULL, red);
					}

					// If computed closest surface point is in margin, compute pressure at that point
					if (distance < include_margin_sq) {
						double pressure                         = s->tri_e_MN().Evaluate(t, bary);
						tactile_state_msg_.sensors[0].values[i] = pressure;
						if (visualize && std::abs(pressure) > 1e-6) {
							tactile_current_scale = std::max(std::abs(pressure), tactile_current_scale);
							initVGeom(mjGEOM_SPHERE, sizeSP, posS, NULL, green);
							if (visualize) {
								float scale          = std::min(std::max(pressure, 0.0), max_pressure) / max_pressure;
								const float color[4] = { scale, 0, 1.0f - scale, 1 };
								mjtNum s             = 0.0005 + scale * 0.002;
								mjtNum size[3]       = { s, s, s };
								initVGeom(mjGEOM_SPHERE, size, pos, NULL, color);
							}

						} else {
							tactile_state_msg_.sensors[0].values[i] = 0;
						}
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
						if (dist(i, j) < include_margin_sq) {
							std::shared_ptr<ContactSurface<double>> s = surfaces[j];
							int t                                     = ts[j];
							Vector3<double> bary                      = barys[j];
							double w                                  = include_margin_sq - dist(i, j);
							double p                                  = w * std::abs(s->tri_e_MN().Evaluate(t, bary));
							pressure += p;
							ws += w;
							if (visualize) {
								normal += p * s->face_normal(t);
								for (int h = 0; h < 3; ++h) {
									posS[h] = surface_points(h, j);
								}
								initVGeom(mjGEOM_SPHERE, sizeSP, posS, NULL, green);
							}
						}
					}
					if (ws > 0) {
						tactile_state_msg_.sensors[0].values[i] = pressure;
						if (visualize) {
							for (int h = 0; h < 3; ++h) {
								pos[h] = taxels_at_M3(h, i);
							}
							float scale          = std::min(std::max(pressure, 0.0), max_pressure) / max_pressure;
							const float color[4] = { scale, 0, 1.0f - scale, 1 };
							mjtNum s             = 0.0005 + scale * 0.002;
							mjtNum size[3]       = { s, s, s };
							initVGeom(mjGEOM_SPHERE, size, pos, NULL, color);
						}
					}
				}
			case MEAN:
				for (int i = 0; i < n; ++i) {
					double ws       = 0;
					double pressure = 0;
					Vector3<double> normal(0, 0, 0);

					if (visualize) {
						for (int h = 0; h < 3; ++h) {
							pos[h] = taxels_at_M3(h, i);
						}
						initVGeom(mjGEOM_SPHERE, sizeSP, pos, NULL, red);
					}

					for (int j = 0; j < m; ++j) {
						if (dist(i, j) < include_margin_sq) {
							std::shared_ptr<ContactSurface<double>> s = surfaces[j];
							int t                                     = ts[j];
							Vector3<double> bary                      = barys[j];
							double p                                  = std::abs(s->tri_e_MN().Evaluate(t, bary));
							pressure += p;
							ws += 1;
							if (visualize) {
								normal += p * s->face_normal(t);
								for (int h = 0; h < 3; ++h) {
									posS[h] = surface_points(h, j);
								}
								initVGeom(mjGEOM_SPHERE, sizeSP, posS, NULL, green);
							}
						}
					}
					if (ws > 0) {
						tactile_state_msg_.sensors[0].values[i] = pressure;
						if (visualize) {
							for (int h = 0; h < 3; ++h) {
								pos[h] = taxels_at_M3(h, i);
							}
							float scale          = std::min(std::max(pressure, 0.0), max_pressure) / max_pressure;
							const float color[4] = { scale, 0, 1.0f - scale, 1 };
							mjtNum s             = 0.0005 + scale * 0.002;
							mjtNum size[3]       = { s, s, s };
							initVGeom(mjGEOM_SPHERE, size, pos, NULL, color);
						}
					}
				}
			case SQUARED:
				for (int i = 0; i < n; ++i) {
					double ws       = 0;
					double pressure = 0;
					for (int j = 0; j < m; ++j) {
						if (dist(i, j) < include_margin_sq) {
							std::shared_ptr<ContactSurface<double>> s = surfaces[j];
							int t                                     = ts[j];
							Vector3<double> bary                      = barys[j];
							double w = pow(std::max(0.0, include_margin - sqrt(dist(i, j))), 2);
							double p = w * std::abs(s->tri_e_MN().Evaluate(t, bary));
							pressure += p;
							ws += 1;
							if (visualize) {
								for (int h = 0; h < 3; ++h) {
									posS[h] = surface_points(h, j);
								}
								initVGeom(mjGEOM_SPHERE, sizeSP, posS, NULL, green);
							}
						}
					}
					if (ws > 0) {
						pressure *= sample_resolution;
						tactile_state_msg_.sensors[0].values[i] = pressure;
					}
					if (visualize) {
						for (int h = 0; h < 3; ++h) {
							pos[h] = taxels_at_M3(h, i);
						}
						float scale          = std::min(std::max(pressure, 0.0), max_pressure) / max_pressure;
						const float color[4] = { scale, 0, 1.0f - scale, 1 };
						mjtNum s             = 0.0005 + scale * 0.002;
						mjtNum size[3]       = { s, s, s };
						initVGeom(mjGEOM_SPHERE, size, pos, NULL, color);
					}
				}
		}
	} else {
		Eigen::Matrix4d M;
		M << data->geom_xmat[9 * id + 0], data->geom_xmat[9 * id + 1], data->geom_xmat[9 * id + 2],
		    data->geom_xpos[3 * id], data->geom_xmat[9 * id + 3], data->geom_xmat[9 * id + 4],
		    data->geom_xmat[9 * id + 5], data->geom_xpos[3 * id + 1], data->geom_xmat[9 * id + 6],
		    data->geom_xmat[9 * id + 7], data->geom_xmat[9 * id + 8], data->geom_xpos[3 * id + 2], 0, 0, 0, 1;
		Eigen::Matrix<double, 4, Eigen::Dynamic> taxels_at_M  = M * taxel_mat;
		Eigen::Matrix<double, 3, Eigen::Dynamic> taxels_at_M3 = taxels_at_M.topRows<3>();

		// If there are no sampled points on the surface fill the sensor message with zeros
		for (int i = 0; i < n; ++i) {
			tactile_state_msg_.sensors[0].values[i] = 0;
			if (visualize) {
				pos[0] = taxels_at_M3(0, i);
				pos[1] = taxels_at_M3(1, i);
				pos[2] = taxels_at_M3(2, i);
				initVGeom(mjGEOM_SPHERE, sizeS, pos, NULL, blue);
			}
		}
	}
}

} // namespace mujoco_ros::contact_surfaces::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::contact_surfaces::sensors::TaxelSensor, mujoco_ros::contact_surfaces::SurfacePlugin)
