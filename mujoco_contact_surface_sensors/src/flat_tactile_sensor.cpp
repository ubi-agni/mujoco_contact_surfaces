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

bool FlatTactileSensor::load(mjModelPtr m, mjDataPtr d)
{
#ifdef BENCHMARK_TACTILE
	benchmark_bvh.name  = "BVH";
	benchmark_mt.name   = "MT";
	benchmark_proj.name = "PROJ";
#endif
	if (TactileSensorBase::load(m, d) && rosparam_config_.hasMember("resolution")) {
		resolution = static_cast<double>(rosparam_config_["resolution"]);

		if (rosparam_config_.hasMember("sampling_resolution")) {
			sampling_resolution = static_cast<int>(rosparam_config_["sampling_resolution"]);
		}

		if (rosparam_config_.hasMember("use_parallel")) {
			use_parallel = static_cast<bool>(rosparam_config_["use_parallel"]);
		}

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
	if (visualize) {
		// reset the visualized geoms
		tactile_running_scale = 0.9 * tactile_running_scale + 0.1 * tactile_current_scale;
		tactile_current_scale = 0.;
	}

#ifdef BENCHMARK_TACTILE
	visualize = false;
	if (skip_update) {
		return;
	}
	projection_update(m, d, geomCollisions);
	mt_update(m, d, geomCollisions);
	bvh_update(m, d, geomCollisions);
	if (print_benchmark && ros::Time::now() - benchmark_bvh.last_report > ros::Duration(0.2)) {
		benchmark_bvh.report();
		benchmark_mt.report();
		benchmark_proj.report();
	}
#else
	bvh_update(m, d, geomCollisions);
#endif
}

void FlatTactileSensor::render_tiles(Eigen::ArrayXXf pressure, mjtNum rot[9], mjtNum origin[3])
{
	if (!visualize)
		return;

	mjtNum size[3]        = { resolution / 2.f, resolution / 2.f, 0.0005 };
	mjtNum sphere_size[3] = { 0.0005, 0.0005, 0.0005 };
	const float rgbaA[4]  = { 1.f, 0.0f, 0.f, 0.3f };
	const float rgbaC[4]  = { 1.0f, 1.0f, 1.f, 0.8f };

	for (int x = 0; x < cx; x++) {
		for (int y = 0; y < cy; y++) {
			double mean_pressure = pressure(x, y);

			tactile_current_scale = std::max(std::abs(mean_pressure), tactile_current_scale);
			float ps              = std::min(std::abs(mean_pressure), tactile_running_scale) / tactile_running_scale;
			const float rgba[4]   = { ps, 0, (1.f - ps), 0.8 };

			mjtNum pos[3] = { origin[0] + x * resolution + (resolution / 2), origin[1] + y * resolution + (resolution / 2),
				               origin[2] };

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

	mjtNum rot[9];
	mju_copy(rot, d->geom_xmat + 9 * id, 9);

	// Negative Z axis is defined as the normal of the flat sensor (sensor points are outside of the object and point
	// inward)
	float3 sensor_normal(static_cast<float>(d->geom_xmat[9 * geomID + 2]),
	                     static_cast<float>(d->geom_xmat[9 * geomID + 5]),
	                     static_cast<float>(-d->geom_xmat[9 * geomID + 8]));
	float3 sensor_center(static_cast<float>(d->geom_xpos[3 * geomID]), static_cast<float>(d->geom_xpos[3 * geomID + 1]),
	                     static_cast<float>(d->geom_xpos[3 * geomID + 2]));

	mjtNum sensor_topleft[3] = { sensor_center[0] - xs, sensor_center[1] - ys, sensor_center[2] + zs };

	BVH bvh[geomCollisions.size()];
	int bvh_idx = 0;

	Eigen::ArrayXXf pressure = Eigen::ArrayXXf::Zero(cx, cy);
#ifdef BENCHMARK_TACTILE
	timer.reset();
	int num_tris = 0;

	for (GeomCollisionPtr gc : geomCollisions) {
		if (gc->g1 == id or gc->g2 == id) {
			bvh[bvh_idx] = BVH(gc->s);
			bvh_idx++;
			num_tris += gc->s->tri_mesh_W().num_triangles();
		}
	}

	double bblas = timer.elapsed();
	if (bvh_idx == 0) {
		benchmark_bvh.add_measure(bblas, 0., bvh_idx, 0., 0., 0., 0.);
		tactile_state_msg_.sensors[0].values.clear();
		tactile_state_msg_.sensors[0].values.resize(cx * cy);
		memcpy(tactile_state_msg_.sensors[0].values.begin(), pressure.data(), cx * cy * sizeof(float));
		render_tiles(pressure, rot, sensor_topleft);
		return; // no contacts with surfaces in this timestep
	}
#else
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
		render_tiles(pressure, rot, sensor_topleft);
		return; // no contacts with surfaces in this timestep
	}
#endif

	TLAS tlas(bvh, bvh_idx);
#ifdef BENCHMARK_TACTILE
	timer.reset();
	tlas.build();
	double btlas = timer.elapsed();
#else
	tlas.build();
#endif
	float rSampling_resolution = resolution / sampling_resolution; // divide once to save time

#ifdef BENCHMARK_TACTILE
	int hits     = 0;
	int num_rays = cx * cy * sampling_resolution * sampling_resolution;
	timer.reset();
#endif

	float *pressure_raw = pressure.data();
	float rmean         = 1. / (sampling_resolution * sampling_resolution);
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

						float3 sensor_point =
						    sensor_center +
						    float3(-xs + x * resolution + i * rSampling_resolution + 0.5 * rSampling_resolution,
						           -ys + y * resolution + j * rSampling_resolution + 0.5 * rSampling_resolution, 1.5 * zs);

						Ray ray;
						ray.d0.data.O = sensor_point;
						ray.d1.data.D = sensor_normal;
						ray.hit.t     = 1e30f;

						tlas.intersect(ray);

						if (ray.hit.t < 1e30f && ray.hit.t > 0.0f) {
							const Eigen::Vector3d bary(1 - ray.hit.u - ray.hit.v, ray.hit.u, ray.hit.v);
							uint tri_idx  = ray.hit.bvh_triangle & 0xFFFFF;
							uint blas_idx = ray.hit.bvh_triangle >> 20;
#ifdef BENCHMARK_TACTILE
							hits++;
#endif
							// We sample points around the sensor in its receptive field and average them
							avg_pressure += tlas.blas[blas_idx].surface->tri_e_MN().Evaluate(tri_idx, bary) * rmean;
						}
					}
				}
			}
			pressure_raw[x + cy * y]                         = avg_pressure;
			tactile_state_msg_.sensors[0].values[x + cy * y] = avg_pressure;
		}
	}
#ifdef BENCHMARK_TACTILE
	double tr = timer.elapsed();
	benchmark_bvh.add_measure(bblas, btlas, bvh_idx, tr, hits, num_rays, num_tris);
#endif
	// ROS_DEBUG_STREAM("BVH BLAS: " << (bblas * 1000.) << "ms, TLAS: " << (btlas*1000.) << "ms, TR: " << (tr*1000.) <<
	// "ms, hits: " << hits);
	render_tiles(pressure, rot, sensor_topleft);
}

void FlatTactileSensor::mt_update(const mjModel *m, mjData *d, const std::vector<GeomCollisionPtr> &geomCollisions)
{
	int id     = geomID;
	double xs  = m->geom_size[3 * id];
	double ys  = m->geom_size[3 * id + 1];
	double zs  = m->geom_size[3 * id + 2];
	double res = resolution;

	// Negative Z axis is defined as the normal of the flat sensor (sensor points are outside of the object and point
	// inward)
	Eigen::Vector3d sensor_normal(d->geom_xmat[9 * geomID + 2], d->geom_xmat[9 * geomID + 5],
	                              -d->geom_xmat[9 * geomID + 8]);

	Eigen::Vector3d sensor_center(d->geom_xpos[3 * geomID], d->geom_xpos[3 * geomID + 1], d->geom_xpos[3 * geomID + 2]);

#ifdef BENCHMARK_TACTILE
	int num_tris = 0;
#endif

	std::vector<std::shared_ptr<ContactSurface<double>>> surfaces;

	for (GeomCollisionPtr gc : geomCollisions) {
		if (gc->g1 == id or gc->g2 == id) {
			surfaces.push_back(gc->s);
#ifdef BENCHMARK_TACTILE
			num_tris += gc->s->tri_mesh_W().num_triangles();
#endif
		}
	}

	const float rgbaInt[4] = { 0.2f, 0.2f, 0.3f, 0.8f };
	const float rgbaC[4]   = { 0.f, 0.0f, 1.f, 0.8f };
	const float rgbaA[4]   = { 1.f, 0.0f, 0.f, 0.3f };
	const float rgbaI[4]   = { 0.f, 1.0f, 0.f, 0.3f };
	mjtNum size[3]         = { 0.001, 0.001, 0.001 };
	mjtNum rot[9];
	mju_copy(rot, d->geom_xmat + 9 * id, 9);

	int x, y, t, i, j;
	std::shared_ptr<ContactSurface<double>> surface;
	Eigen::ArrayXXd pressure = Eigen::ArrayXXd::Zero(sampling_resolution * cx, sampling_resolution * cy);

#ifdef BENCHMARK_TACTILE
	int hits     = 0;
	int num_rays = cx * cy * sampling_resolution * sampling_resolution * surfaces.size();
	timer.reset();
#pragma omp target map(tofrom                         \
                       : pressure) map(from           \
                                       : hits) map(to \
                                                   : x, y, t, i, j, surfaces, sensor_center) if (use_parallel)
#else
#pragma omp target map(tofrom : pressure) map(to : x, y, t, i, j, surfaces, sensor_center) if (use_parallel)
#endif
	{
		for (std::shared_ptr<ContactSurface<double>> surface : surfaces) {
			// #pragma omp teams distribute parallel for collapse(5) if(use_parallel)
			for (x = 0; x < cx; x++) {
				for (y = 0; y < cy; y++) {
					for (i = 0; i < sampling_resolution; i++) {
						for (j = 0; j < sampling_resolution; j++) {
							for (t = 0; t < surface->tri_mesh_W().num_triangles(); t++) {
								const Eigen::Vector3d sensor_offset(
								    -xs + x * resolution + i * resolution / sampling_resolution +
								        0.5 * resolution / sampling_resolution,
								    -ys + y * resolution + j * resolution / sampling_resolution +
								        0.5 * resolution / sampling_resolution,
								    1.5 * zs);
								const Eigen::Vector3d sensor_point = sensor_center + sensor_offset;

								mjtNum pos[3] = { sensor_point[0], sensor_point[1], sensor_point[2] };

								const auto &tri = surface->tri_mesh_W().element(t);

								const Vector3<double> &v0 = surface->tri_mesh_W().vertex(tri.vertex(0));
								const Vector3<double> &v1 = surface->tri_mesh_W().vertex(tri.vertex(1));
								const Vector3<double> &v2 = surface->tri_mesh_W().vertex(tri.vertex(2));
								const Vector3<double> &n  = surface->tri_mesh_W().face_normal(t);

								Eigen::Vector3d intersection_point;

								Eigen::Vector3d edge1 = v1 - v0;
								Eigen::Vector3d edge2 = v2 - v0;
								Eigen::Vector3d h     = sensor_normal.cross(edge2);
								float a, f, u, v;
								a = edge1.dot(h);

								if (a > -0.0000001 && a < 0.0000001) {
									// sensor_point is parallel to this triangle
									continue;
								}

								f                 = 1.0 / a;
								Eigen::Vector3d s = sensor_point - v0;
								u                 = f * s.dot(h);

								if (u < 0.0 || u > 1.0) {
									// sensor_point is outside of the triangle
									continue;
								}

								Eigen::Vector3d q = s.cross(edge1);
								v                 = f * sensor_normal.dot(q);

								if (v < 0.0 || u + v > 1.0) {
									// sensor_point is outside of the triangle
									continue;
								}

								float r = f * edge2.dot(q);

								if (r > 0.0000001) {
									// sensor_point is on the triangle
									intersection_point = sensor_point + r * sensor_normal;
								} else {
									// sensor_point is not on the triangle
									continue;
								}
#ifdef BENCHMARK_TACTILE
								hits++;
#endif
								const Vector3<double> bary(1 - u - v, u, v);
								pressure(x * sampling_resolution + i, y * sampling_resolution + j) =
								    surface->tri_e_MN().Evaluate(t, bary) * surface->area(t);
							}
						}
					}
				}
			}
		}
	}
#ifdef BENCHMARK_TACTILE
	double tr = timer.elapsed();
	benchmark_mt.add_measure(0, 0, 0, tr, hits, num_rays, num_tris);
	// ROS_DEBUG_STREAM("MT BLAS: " << 0 << ", TLAS: " << 0 << ", TR: " << (tr*1000.) << "ms, hits: " << hits);
#endif

	for (int x = 0; x < cx; x++) {
		for (int y = 0; y < cy; y++) {
			double mean_pressure = 0.0;
#pragma omp parallel for collapse(2) reduction(+ : mean_pressure)
			for (int i = 0; i < sampling_resolution; i++) {
				for (int j = 0; j < sampling_resolution; j++) {
					mean_pressure += pressure(sampling_resolution * x + i, sampling_resolution * y + j);
				}
			}

			mean_pressure /= sampling_resolution * sampling_resolution;
			tactile_state_msg_.sensors[0].values[x * cx + y] = mean_pressure;

			if (visualize) {
				mjvGeom *g = vGeoms + n_vGeom++;

				mjtNum pos[3] = { d->geom_xpos[3 * geomID] - xs + x * resolution + (resolution / 2),
					               d->geom_xpos[3 * geomID + 1] - ys + y * resolution + (resolution / 2),
					               d->geom_xpos[3 * geomID + 2] + zs };
				mjv_initGeom(g, mjGEOM_SPHERE, size, pos, rot, rgbaC);

				if (mean_pressure > 0.0) {
					g               = vGeoms + n_vGeom++;
					mjtNum sizeA[3] = { 0.001, 0.001, 0.1 * mean_pressure };
					mjv_initGeom(g, mjGEOM_ARROW, sizeA, pos, rot, rgbaA);
				}
			}
		}
	}
}

void FlatTactileSensor::projection_update(const mjModel *m, mjData *d,
                                          const std::vector<GeomCollisionPtr> &geomCollisions)
{
	int id     = geomID;
	double xs  = m->geom_size[3 * id];
	double ys  = m->geom_size[3 * id + 1];
	double zs  = m->geom_size[3 * id + 2];
	double res = resolution;
	Eigen::Matrix4d M, Minv, Mback;
	M << d->geom_xmat[9 * id + 0], d->geom_xmat[9 * id + 1], d->geom_xmat[9 * id + 2], d->geom_xpos[3 * id],
	    d->geom_xmat[9 * id + 3], d->geom_xmat[9 * id + 4], d->geom_xmat[9 * id + 5], d->geom_xpos[3 * id + 1],
	    d->geom_xmat[9 * id + 6], d->geom_xmat[9 * id + 7], d->geom_xmat[9 * id + 8], d->geom_xpos[3 * id + 2], 0, 0, 0,
	    1;

	Minv = M.inverse();
	Minv.col(3) << Minv.col(3) + Eigen::Vector4d(xs, ys, -zs, 0);
	if (visualize) {
		Mback = Minv.inverse();
	}
	std::vector<double> pressure[cx][cy];

	mjtNum size[3] = { res / 2, res / 2, 0.001 };
	mjtNum rot[9];

	mju_copy(rot, d->geom_xmat + 9 * id, 9);

#ifdef BENCHMARK_TACTILE
	int hits     = 0;
	int num_rays = 0;
	int num_tris = 0;
	timer.reset();
#endif
	for (GeomCollisionPtr gc : geomCollisions) {
		if (gc->g1 == id or gc->g2 == id) {
			std::shared_ptr<ContactSurface<double>> s = gc->s;

			// prepare caches
			const int n = s->tri_mesh_W().num_elements();
#ifdef BENCHMARK_TACTILE
			num_tris += n;
#endif
			int t, i, st0, st1, st2, st, x, y;

			// #pragma omp target map(from: pressure[0:cx][0:cy], hits) map(to: s, n, t, st0, st1, st2, st, x, y) if(false)
			// //if(use_parallel)
			{
				// #pragma omp teams distribute parallel for if(false) //if(use_parallel)
				for (t = 0; t < n; ++t) {
					// project points onto 2d sensor plane
					std::vector<Eigen::Vector2d> tpoints = {};
					const auto &element                  = s->tri_mesh_W().element(t);
					for (i = 0; i < element.num_vertices(); ++i) {
						int v                     = element.vertex(i);
						const Vector3<double> &vp = s->tri_mesh_W().vertex(v);
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
#ifdef BENCHMARK_TACTILE
							num_rays++;
#endif
							const Vector3<double> bary(a, (1 - a) * (1 - b), (1 - a) * b);
							Eigen::Vector2d p = bary[0] * tpoints[0] + bary[1] * tpoints[1] + bary[2] * tpoints[2];
							if (p[0] > 0 && p[0] < 2 * xs && p[1] > 0 && p[1] < 2 * ys) {
#ifdef BENCHMARK_TACTILE
								hits++;
#endif
								x = (int)std::floor(p[0] / res);
								y = (int)std::floor(p[1] / res);
								pressure[x][y].push_back(s->tri_e_MN().Evaluate(t, bary) * s->area(t));
							}
						}
					}
				}
			}
		}
	}
#ifdef BENCHMARK_TACTILE
	float tr = timer.elapsed() * 1000;
	benchmark_proj.add_measure(0, 0, 0, tr, hits, num_rays, num_tris);
	// ROS_DEBUG_STREAM("PROJ BLAS: " << 0 << ", TLAS: " << 0 << ", TR: " << (tr*1000.) << "ms, hits: " << hits);
#endif

	for (int x = 0; x < cx; ++x) {
		for (int y = 0; y < cy; ++y) {
			int nt = pressure[x][y].size();
			if (nt > 0) {
				double mp = 0;

				for (int i = 0; i < nt; ++i) {
					// int t = tris[x][y][i];
					mp += pressure[x][y][i];
				}

				double p0                                        = mp / nt;
				tactile_state_msg_.sensors[0].values[x * cx + y] = p0;
				if (visualize) {
					tactile_current_scale = std::max(std::abs(p0), tactile_current_scale);
					float ps              = std::min(std::abs(p0), tactile_running_scale) / tactile_running_scale;

					const float rgba[4] = { ps, 0, (1.f - ps), 0.8 };
					Eigen::Vector4d dp  = Mback * Eigen::Vector4d(x * res + res / 2, y * res + res / 2, 0, 1);
					mjtNum pos[3]       = { dp[0], dp[1], dp[2] };

					mjvGeom *g = vGeoms + n_vGeom++;
					mjv_initGeom(g, mjGEOM_BOX, size, pos, rot, rgba);
				}
			} else {
				tactile_state_msg_.sensors[0].values[x * cx + y] = 0;
			}
		}
	}
}

} // namespace mujoco_ros::contact_surfaces::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::contact_surfaces::sensors::FlatTactileSensor,
                       mujoco_ros::contact_surfaces::SurfacePlugin)
