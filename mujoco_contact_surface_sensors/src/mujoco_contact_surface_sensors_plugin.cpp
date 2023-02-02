/**
 *
 * Parts of the code used to evaluate the contact surfaces
 * (namely the methods evaluateContactSurfaces and passive_cb)
 * are based on code of Drake which is licensed as follows:
 *
 * All components of Drake are licensed under the BSD 3-Clause License
 * shown below. Where noted in the source code, some portions may
 * be subject to other permissive, non-viral licenses.
 *
 * Copyright 2012-2022 Robot Locomotion Group @ CSAIL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the Massachusetts Institute of Technology nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * The rest is licensed under the following license:
 *
 * Software License Agreement (BSD 3-Clause License)
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
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
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
 */

/* Authors: Florian Patzelt*/

#include <mujoco_contact_surface_sensors/mujoco_contact_surface_sensors_plugin.h>

#include <pluginlib/class_list_macros.h>

namespace mujoco_contact_surface_sensors {

bool MujocoContactSurfaceSensorsPlugin::load(mjModelPtr m, mjDataPtr d)
{
	MujocoContactSurfacesPlugin::load(m, d);
	parseROSParam();

	// TODO add warning if both this and MujocoContactSurfaces are loaded
	ROS_INFO_NAMED("mujoco_contact_surfaces", "Loaded mujoco_contact_surface_sensors");
	return true;
}

void MujocoContactSurfaceSensorsPlugin::parseROSParam()
{
	if (rosparam_config_.hasMember("TactileArrays") &&
	    rosparam_config_["TactileArrays"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
		auto tactile_arrays = rosparam_config_["TactileArrays"];
		int n               = rosparam_config_["TactileArrays"].size();

		for (int i = 0; i < n; ++i) {
			auto ta = tactile_arrays[i];
			if (ta.getType() == XmlRpc::XmlRpcValue::TypeStruct && ta.hasMember("updateRate") &&
			    ta.hasMember("geomName") && ta.hasMember("topicName") && ta.hasMember("resolution") &&
			    ta.hasMember("sensorName")) {
				std::string geomName   = static_cast<std::string>(ta["geomName"]);
				std::string topicName  = static_cast<std::string>(ta["topicName"]);
				std::string sensorName = static_cast<std::string>(ta["sensorName"]);
				double resolution      = static_cast<double>(ta["resolution"]);
				double updateRate      = static_cast<double>(ta["updateRate"]);
				ros::Duration updatePeriod(1.0 / updateRate);

				int id = mj_name2id(m_.get(), mjOBJ_GEOM, geomName.c_str());
				if (id >= 0) {
					TactileSensor *ts = new TactileSensor();
					ts->geomID        = id;
					ts->geomName      = geomName;
					ts->sensorName    = sensorName;
					ts->resolution    = resolution;
					ts->updatePeriod  = updatePeriod;
					ts->updateRate    = updateRate;
					ts->topicName     = topicName;
					ts->publisher     = node_handle_->advertise<tactile_msgs::TactileState>(topicName, 1, true);
					ts->lastUpdate    = ros::Time();
					double xs         = m_->geom_size[3 * id];
					double ys         = m_->geom_size[3 * id + 1];
					ts->cx            = (int)(2 * xs / resolution);
					ts->cy            = (int)(2 * ys / resolution);
					ts->vGeoms        = new mjvGeom[ts->cx * ts->cy];
					ROS_INFO_STREAM_NAMED("mujoco_contact_surfaces",
					                      "Found tactile sensor: " << sensorName << " " << ts->cx << "x" << ts->cy);
					sensor_msgs::ChannelFloat32 channel;
					channel.values.resize(ts->cx * ts->cy);
					channel.name = sensorName;
					ts->tactile_state_msg_.sensors.push_back(channel);
					tactileSensors.push_back(ts);
				}
			}
		}
	}
}

void MujocoContactSurfaceSensorsPlugin::passive_cb(const mjModel *m, mjData *d)
{
	if (visualizeContactSurfaces) {
		// reset the visualized geoms
		tactile_running_scale = 0.9 * tactile_running_scale + 0.1 * tactile_current_scale;
		tactile_current_scale = 0.;
	}
	for (TactileSensor *ts : tactileSensors) {
		auto now = ros::Time::now();
		if (now < ts->lastUpdate) // reset lastUpdate after jump back in time
			ts->lastUpdate = ros::Time();

		if (now - ts->lastUpdate >= ts->updatePeriod) {
			ts->lastUpdate = now;
			ts->n_vGeom    = 0;
			int id         = ts->geomID;
			int cx         = ts->cx;
			int cy         = ts->cy;
			double xs      = m->geom_size[3 * id];
			double ys      = m->geom_size[3 * id + 1];
			double zs      = m->geom_size[3 * id + 2];
			double res     = ts->resolution;

			Eigen::Matrix4d M, Minv, Mback;
			M << d->geom_xmat[9 * id + 0], d->geom_xmat[9 * id + 1], d->geom_xmat[9 * id + 2], d->geom_xpos[3 * id],
			    d->geom_xmat[9 * id + 3], d->geom_xmat[9 * id + 4], d->geom_xmat[9 * id + 5], d->geom_xpos[3 * id + 1],
			    d->geom_xmat[9 * id + 6], d->geom_xmat[9 * id + 7], d->geom_xmat[9 * id + 8], d->geom_xpos[3 * id + 2], 0,
			    0, 0, 1;

			Minv = M.inverse();
			Minv.col(3) << Minv.col(3) + Eigen::Vector4d(xs, ys, -zs, 0);

			Mback = Minv.inverse();
			std::vector<double> pressure[cx][cy];

			mjtNum size[3] = { res / 2, res / 2, 0.001 };
			mjtNum rot[9];
			for (int i = 0; i < 9; ++i) {
				rot[i] = d->geom_xmat[9 * id + i];
			}

			for (GeomCollision *gc : geomCollisions) {
				if (gc->g1 == id or gc->g2 == id) {
					ContactSurface<double> s = gc->s;
					auto mesh                = s.tri_mesh_W();

					// prepare caches
					const int n = mesh.num_elements();

					// get geom transformation
					// std::vector<int> tris[cx][cy];
					// std::vector<Vector3<double>> barys[cx][cy];
					for (int t = 0; t < n; ++t) {
						// project points onto 2d sensor plane
						std::vector<Eigen::Vector2d> tpoints = {};
						auto element                         = mesh.element(t);
						for (int i = 0; i < element.num_vertices(); ++i) {
							int v                     = element.vertex(i);
							const Vector3<double> &vp = mesh.vertex(v);
							const Eigen::Vector4d vpe(vp[0], vp[1], vp[2], 1);
							Eigen::Vector4d vpp = Minv * vpe;

							tpoints.push_back(Eigen::Vector2d(vpp[0], vpp[1]));
						}
						int st0 = (int)((tpoints[1] - tpoints[0]).norm() / res * 2.) + 1;
						int st1 = (int)((tpoints[2] - tpoints[0]).norm() / res * 2.) + 1;
						int st2 = (int)((tpoints[2] - tpoints[0]).norm() / res * 2.) + 1;
						int st  = std::max(st0, st1);
						for (double a = 0; a <= 1; a += 1. / st) {
							for (double b = 0; b <= 1; b += 1. / st2) {
								const Vector3<double> bary(a, (1 - a) * (1 - b), (1 - a) * b);
								Eigen::Vector2d p = bary[0] * tpoints[0] + bary[1] * tpoints[1] + bary[2] * tpoints[2];
								if (p[0] > 0 && p[0] < 2 * xs && p[1] > 0 && p[1] < 2 * ys) {
									int x = (int)std::floor(p[0] / res);
									int y = (int)std::floor(p[1] / res);
									// barys[x][y].push_back(bary);
									// tris[x][y].push_back(t);
									pressure[x][y].push_back(s.tri_e_MN().Evaluate(t, bary) * s.area(t));
								}
							}
						}
					}
				}
			}
			for (int x = 0; x < cx; ++x) {
				for (int y = 0; y < cy; ++y) {
					int nt = pressure[x][y].size();
					if (nt > 0) {
						double mp = 0;

						for (int i = 0; i < nt; ++i) {
							// int t = tris[x][y][i];
							mp += pressure[x][y][i];
						}

						double p0                                            = mp / nt;
						ts->tactile_state_msg_.sensors[0].values[x * cx + y] = p0;

						// ROS_INFO_STREAM_NAMED("mujoco_contact_surfaces", "P*A: " << p0);

						tactile_current_scale = std::max(std::abs(p0), tactile_current_scale);
						float ps              = std::min(std::abs(p0), tactile_running_scale) / tactile_running_scale;

						const float rgba[4] = { ps, 0, (1.f - ps), 0.8 };
						Eigen::Vector4d dp  = Mback * Eigen::Vector4d(x * res + res / 2, y * res + res / 2, 0, 1);
						mjtNum pos[3]       = { dp[0], dp[1], dp[2] };

						mjvGeom *g = ts->vGeoms + ts->n_vGeom++;
						mjv_initGeom(g, mjGEOM_BOX, size, pos, rot, rgba);
					} else {
						ts->tactile_state_msg_.sensors[0].values[x * cx + y] = 0;
					}
				}
			}
			if (ts->publisher.getNumSubscribers() > 0) {
				ts->tactile_state_msg_.header.stamp = now;
				ts->publisher.publish(ts->tactile_state_msg_);
			}
		}
	}
	MujocoContactSurfacesPlugin::passive_cb(m, d);
}

void MujocoContactSurfaceSensorsPlugin::renderCallback(mjModelPtr model, mjDataPtr data, mjvScene *scene)
{
	MujocoContactSurfacesPlugin::renderCallback(model, data, scene);
	if (visualizeContactSurfaces) {
		for (TactileSensor *ts : tactileSensors) {
			for (int i = 0; i < ts->n_vGeom && scene->ngeom < scene->maxgeom; ++i) {
				scene->geoms[scene->ngeom++] = ts->vGeoms[i];
			}
		}
	}
}

} // namespace mujoco_contact_surface_sensors

PLUGINLIB_EXPORT_CLASS(mujoco_contact_surface_sensors::MujocoContactSurfaceSensorsPlugin,
                       mujoco_contact_surfaces::MujocoContactSurfacesPlugin)
