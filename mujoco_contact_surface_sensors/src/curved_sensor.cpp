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

#include <mujoco_contact_surface_sensors/curved_sensor.h>

#include <pluginlib/class_list_macros.h>
#include <random>
#include <vcg/complex/complex.h>

#include <vcg/complex/algorithms/point_sampling.h>
#include <vcg/complex/algorithms/create/platonic.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

using namespace std;
using namespace vcg;

class MyEdge;
class MyFace;
class MyVertex;
struct MyUsedTypes : public UsedTypes<Use<MyVertex>::AsVertexType, Use<MyEdge>::AsEdgeType, Use<MyFace>::AsFaceType>
{};

class MyVertex : public Vertex<MyUsedTypes, vertex::Coord3f, vertex::Normal3f, vertex::BitFlags>
{};
class MyFace : public Face<MyUsedTypes, face::FFAdj, face::Normal3f, face::VertexRef, face::BitFlags>
{};
class MyEdge : public Edge<MyUsedTypes>
{};
class MyMesh : public tri::TriMesh<std::vector<MyVertex>, std::vector<MyFace>, std::vector<MyEdge>>
{};

namespace mujoco_ros::contact_surfaces::sensors {
using namespace drake;
using namespace drake::geometry;

void CurvedSensor::sample_triangle(int n, Vector3<double> A, Vector3<double> B, Vector3<double> C, double area,
                                   std::vector<Vector3<double>> &samples, std::vector<double> &areas)
{
	switch (n) {
		case 0:
			/* code */
			break;
		case 1:
			samples.push_back((A + B + C) / 3.0);
			areas.push_back(area);
			break;
		default:
			double a = (B - C).squaredNorm();
			double b = (A - C).squaredNorm();
			double c = (A - B).squaredNorm();
			if (a > b) {
				if (a > c) {
					// a
					sample_triangle(n / 2 + n % 2, B, (B + C) / 2.0, A, area / 2.0, samples, areas);
					sample_triangle(n / 2, C, (B + C) / 2.0, A, area / 2.0, samples, areas);
				} else {
					// c
					sample_triangle(n / 2 + n % 2, B, (B + A) / 2.0, C, area / 2.0, samples, areas);
					sample_triangle(n / 2, A, (B + A) / 2.0, C, area / 2.0, samples, areas);
				}
			} else {
				if (b > c) {
					// b
					sample_triangle(n / 2 + n % 2, C, (C + A) / 2.0, B, area / 2.0, samples, areas);
					sample_triangle(n / 2, A, (C + A) / 2.0, B, area / 2.0, samples, areas);
				} else {
					// c
					sample_triangle(n / 2 + n % 2, B, (B + A) / 2.0, C, area / 2.0, samples, areas);
					sample_triangle(n / 2, A, (B + A) / 2.0, C, area / 2.0, samples, areas);
				}
			}
			break;
	}
}

bool CurvedSensor::load(const mjModel *m, mjData *d)
{
	if (TactileSensorBase::load(m, d) && rosparam_config_.hasMember("taxels") && rosparam_config_.hasMember("method") &&
	    rosparam_config_.hasMember("include_margin") && rosparam_config_.hasMember("sample_resolution")) {
		include_margin    = static_cast<double>(rosparam_config_["include_margin"]);
		include_margin_sq = include_margin * include_margin;
		sample_resolution = static_cast<double>(rosparam_config_["sample_resolution"]);

		if (not rosparam_config_.hasMember("method")) {
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
			taxel_point_mat  = Eigen::Matrix<double, 4, Eigen::Dynamic>(4, n);
			taxel_normal_mat = Eigen::Matrix<double, 4, Eigen::Dynamic>::Zero(4, n);

			for (int i = 0; i < n; ++i) {
				if (taxel_array[i].getType() == XmlRpc::XmlRpcValue::TypeArray && taxel_array[i].size() == 3) {
					Vector3<double> t(static_cast<double>(taxel_array[i][0]), static_cast<double>(taxel_array[i][1]),
					                  static_cast<double>(taxel_array[i][2]));
					taxel_point_mat.col(i) << t[0], t[1], t[2], 1;
				} else {
					return false;
				}
			}
		} else {
			return false;
		}

		if (rosparam_config_.hasMember("normals")) {
			auto normal_array = rosparam_config_["normals"];
			if (normal_array.getType() == XmlRpc::XmlRpcValue::TypeArray && normal_array.size() > 0) {
				int n = normal_array.size();

				for (int i = 0; i < n; ++i) {
					if (normal_array[i].getType() == XmlRpc::XmlRpcValue::TypeArray && normal_array[i].size() == 3) {
						Vector3<double> t(static_cast<double>(normal_array[i][0]), static_cast<double>(normal_array[i][1]),
						                  static_cast<double>(normal_array[i][2]));
						t.normalize();
						taxel_normal_mat.col(i) << t[0], t[1], t[2], 0;
					} else {
						ROS_ERROR_STREAM_NAMED("mujoco_contact_surface_sensors", "Normal vector must have length 3!");
						return false;
					}
				}
			} else {
				ROS_ERROR_STREAM_NAMED("mujoco_contact_surface_sensors", "Empty normal array or wrong type!");
				return false;
			}
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
				Eigen::Matrix4d M;
				mjtNum *R = mj_stackAlloc(d, 9);
				mjtNum *p = mj_stackAlloc(d, 3);
				mjtNum *q = mj_stackAlloc(d, 4);

				mju_negPose(p, q, p0, q0);

				mju_quat2Mat(R, q);

				M << R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1;

				taxel_point_mat  = M * taxel_point_mat;
				taxel_normal_mat = M * taxel_normal_mat;
				mjFREESTACK;
			}
		}
		int n = taxel_point_mat.cols();
		ROS_INFO_STREAM_NAMED("mujoco_contact_surface_sensors",
		                      "Found taxel sensor '" << sensorName << "' with " << n << " taxels.");
		sensor_msgs::ChannelFloat32 channel;

		channel.values.resize(n);
		channel.name = sensorName;
		tactile_state_msg_.sensors.push_back(channel);

		surface_idx    = std::vector<std::vector<int>>(n);
		surface_weight = std::vector<std::vector<double>>(n);

		// Sample on mesh surface
		// TODO implement methods to sample on primitive geometric surfaces
		if (m->geom_type[geomID] == mjGEOM_MESH) {
			int geom_dataid = m->geom_dataid[geomID];
			if (geom_dataid >= 0) {
				int nv    = m->mesh_vertnum[geom_dataid];
				int nf    = m->mesh_facenum[geom_dataid];
				int v_adr = 3 * m->mesh_vertadr[geom_dataid];
				int f_adr = 3 * m->mesh_faceadr[geom_dataid];
				if (nv > 0 && nf > 0 && v_adr >= 0 && f_adr >= 0) {
					std::vector<SurfaceTriangle> triangles;
					std::vector<Vector3<double>> vertices;

					MyMesh mesh;
					tri::Allocator<MyMesh>::AddVertices(mesh, nv);
					tri::Allocator<MyMesh>::AddFaces(mesh, nf);

					for (int i = 0; i < nv; ++i) {
						int v = v_adr + 3 * i;
						vertices.push_back(Vector3<double>(m->mesh_vert[v], m->mesh_vert[v + 1], m->mesh_vert[v + 2]));
						mesh.vert[i].P() = MyMesh::CoordType(m->mesh_vert[v], m->mesh_vert[v + 1], m->mesh_vert[v + 2]);
					}
					for (int i = 0; i < nf; ++i) {
						int f = f_adr + 3 * i;
						triangles.push_back(SurfaceTriangle(m->mesh_face[f], m->mesh_face[f + 1], m->mesh_face[f + 2]));
						mesh.face[i].V(0) = &mesh.vert[m->mesh_face[f]];
						mesh.face[i].V(1) = &mesh.vert[m->mesh_face[f + 1]];
						mesh.face[i].V(2) = &mesh.vert[m->mesh_face[f + 2]];
					}
					TriangleSurfaceMesh<double> *sm =
					    new TriangleSurfaceMesh<double>(std::move(triangles), std::move(vertices));

					tri::SurfaceSampling<MyMesh, tri::TrivialSampler<MyMesh>>::SamplingRandomGenerator().initialize(42);

					//----------------------------------------------------------------------
					// Basic Sample of a mesh surface
					// Build a point cloud with points with a plain poisson disk distribution

					std::vector<Point3f> pointVec;
					float rad     = 0;
					int sampleNum = (int)sample_resolution * sm->total_area();
					std::cout << "sampleNum: " << sampleNum << std::endl;
					tri::PoissonSampling<MyMesh>(mesh, pointVec, sampleNum, rad);

					int m = pointVec.size();

					std::vector<Vector3<double>> spoints;
					std::vector<Vector3<double>> snormals;
					std::vector<double> sareas;
					std::vector<std::vector<int>> tidx;

					for (int i = 0; i < m; ++i) {
						auto p = Vector3<double>(pointVec[i][0], pointVec[i][1], pointVec[i][2]);

						tidx.push_back(std::vector<int>());

						for (int t = 0; t < nf; ++t) {
							auto element = sm->element(t);
							auto a       = sm->vertex(element.vertex(0));
							auto b       = sm->vertex(element.vertex(1));
							auto c       = sm->vertex(element.vertex(2));
							auto v2      = p - a;
							if (std::abs(v2.dot(sm->face_normal(t))) > 1e-7)
								continue;

							auto v0 = b - a;
							auto v1 = c - a;

							float d00 = v0.dot(v0);
							float d01 = v0.dot(v1);
							float d11 = v1.dot(v1);

							float d20   = v2.dot(v0);
							float d21   = v2.dot(v1);
							float denom = d00 * d11 - d01 * d01;

							float v = (d11 * d20 - d01 * d21) / denom;
							float w = (d00 * d21 - d01 * d20) / denom;
							float u = 1.0f - v - w;
							if (v >= 0.0 && v <= 1.0 && w >= 0.0 && w <= 1.0 && u >= 0.0 && u <= 1.0 &&
							    std::abs(v + w + u - 1.0) < 1e-10) {
								snormals.push_back(sm->face_normal(t));
								sareas.push_back(sm->total_area() / m);
								tidx[i].push_back(t);
								break;
							}
						}

						if (snormals.size() > spoints.size()) {
							spoints.push_back(p);
						}
					}

					Eigen::Matrix<double, 3, Eigen::Dynamic> surface_points0(3, m);
					for (int i = 0; i < m; ++i) {
						surface_points0.col(i) << spoints[i];
					}
					Eigen::Matrix<double, 3, Eigen::Dynamic> taxel_mat3 = taxel_point_mat.topRows<3>();
					// Compute distance matrix between taxel positions and surface points
					Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> dist(n, m);
					dist = ((-2 * taxel_mat3.transpose() * surface_points0).colwise() +
					        taxel_mat3.colwise().squaredNorm().transpose())
					           .rowwise() +
					       surface_points0.colwise().squaredNorm();

					std::vector<int> close_points_idx;
					for (int j = 0; j < m; ++j) {
						bool added = false;
						for (int i = 0; i < n; ++i) {
							auto normal = taxel_normal_mat.topRows<3>().col(i);
							if (dist(i, j) < include_margin_sq &&
							    (normal.squaredNorm() == 0 || acos(normal.dot(snormals[j])) < 45 * M_PI / 180.)) {
								if (!added) {
									close_points_idx.push_back(j);
									added = true;
								}
								surface_idx[i].push_back(close_points_idx.size() - 1);
								surface_weight[i].push_back(
								    pow(std::max(0.0, include_margin - sqrt(dist(i, j))), 2)); //*	  sareas[j]);
							}
						}
					}
					int nc             = close_points_idx.size();
					surface_point_mat  = Eigen::Matrix<double, 4, Eigen::Dynamic>(4, nc);
					surface_normal_mat = Eigen::Matrix<double, 4, Eigen::Dynamic>(4, nc);

					for (int i = 0; i < nc; ++i) {
						auto t      = spoints[close_points_idx[i]];
						auto normal = snormals[close_points_idx[i]];
						surface_point_mat.col(i) << t[0], t[1], t[2], 1;
						surface_normal_mat.col(i) << normal[0], normal[1], normal[2], 0;
					}
				}
			}
			toogle_service =
			    node_handle_.advertiseService("toogle_contact_sensor_visualization", &CurvedSensor::toogle_vis, this);
			return true;
		}
	}
	return false;
}

bool CurvedSensor::toogle_vis(EmptyRequest &req, EmptyResponse &res)
{
	visualization_mode += 1;
	return true;
}

void CurvedSensor::internal_update(const mjModel *model, mjData *data,
                                   const std::vector<GeomCollisionPtr> &geomCollisions)
{
	int id = geomID;
	Eigen::Matrix4d M;
	M << data->geom_xmat[9 * id + 0], data->geom_xmat[9 * id + 1], data->geom_xmat[9 * id + 2], data->geom_xpos[3 * id],
	    data->geom_xmat[9 * id + 3], data->geom_xmat[9 * id + 4], data->geom_xmat[9 * id + 5],
	    data->geom_xpos[3 * id + 1], data->geom_xmat[9 * id + 6], data->geom_xmat[9 * id + 7],
	    data->geom_xmat[9 * id + 8], data->geom_xpos[3 * id + 2], 0, 0, 0, 1;
	Eigen::Matrix<double, 4, Eigen::Dynamic> surface_at_M         = M * surface_point_mat;
	Eigen::Matrix<double, 3, Eigen::Dynamic> surface_at_M3        = surface_at_M.topRows<3>();
	Eigen::Matrix<double, 4, Eigen::Dynamic> surface_normal_at_M  = M * surface_normal_mat;
	Eigen::Matrix<double, 3, Eigen::Dynamic> surface_normal_at_M3 = surface_normal_at_M.topRows<3>();

	int n = taxel_point_mat.cols();
	int m = surface_normal_mat.cols();

	BVH bvh[geomCollisions.size()];
	int bvh_idx = 0;
	std::vector<double> pressure(n);

	const int n_cols = 16;
	mjtNum sizeA[3]  = { 0.0002, 0.0002, 0.005 };

	float colors[n_cols][4] = {
		{ 0.9019607901573181, 0.09803921729326248, 0.29411765933036804, 1 },
		{ 0.23529411852359772, 0.7058823704719543, 0.29411765933036804, 1 },
		{ 1.0, 0.8823529481887817, 0.09803921729326248, 1 },
		{ 0.0, 0.5098039507865906, 0.7843137383460999, 1 },
		{ 0.9607843160629272, 0.5098039507865906, 0.1882352977991104, 1 },
		{ 0.27450981736183167, 0.9411764740943909, 0.9411764740943909, 1 },
		{ 0.9411764740943909, 0.19607843458652496, 0.9019607901573181, 1 },
		{ 0.9803921580314636, 0.7450980544090271, 0.8313725590705872, 1 },
		{ 0.0, 0.501960813999176, 0.501960813999176, 1 },
		{ 0.8627451062202454, 0.7450980544090271, 1.0, 1 },
		{ 0.6666666865348816, 0.4313725531101227, 0.1568627506494522, 1 },
		{ 1.0, 0.9803921580314636, 0.7843137383460999, 1 },
		{ 0.501960813999176, 0.0, 0.0, 1 },
		{ 0.6666666865348816, 1.0, 0.7647058963775635, 1 },
		{ 0.0, 0.0, 0.501960813999176, 1 },
		{ 0.501960813999176, 0.501960813999176, 0.501960813999176, 1 },
	};

	for (GeomCollisionPtr gc : geomCollisions) {
		if (gc->g1 == id or gc->g2 == id) {
			bvh[bvh_idx] = BVH(gc->s);
			bvh_idx++;
		}
	}
	double t_array[m];
	for (int j = 0; j < m; ++j) {
		t_array[j] = 0;
	}
	if (bvh_idx == 0) {
		for (int i = 0; i < n; ++i) {
			tactile_state_msg_.sensors[0].values[i] = 0;
			pressure[i]                             = 0;
		}
	} else {
		TLAS tlas(bvh, bvh_idx);
		tlas.build();

		for (int i = 0; i < n; ++i) {
			pressure[i] = 0;
			for (int j0 = 0; j0 < surface_idx[i].size(); ++j0) {
				int j = surface_idx[i][j0];

				float3 normal = float3(surface_normal_at_M3(0, j), surface_normal_at_M3(1, j), surface_normal_at_M3(2, j));
				float3 surface_point = float3(surface_at_M3(0, j), surface_at_M3(1, j), surface_at_M3(2, j));

				Ray ray;
				ray.d0.data.O = surface_point + normal * 1e-8;
				ray.d1.data.D = -normal;
				ray.hit.t     = 1e30f;

				tlas.intersect(ray);
				// Instead of using 1e30f, we use the maximum distance to the sensor geom z-axis centroid (local
				// frame) to prevent contacts from the wrong side of the sensor to be considered.
				// TODO(dleins): Maybe we should move the maximum distance as a parameter to the ray
				// intersection functions, to directly discard AABBs that are too far away and avoid more
				// intersection tests

				if (ray.hit.t < include_margin && ray.hit.t > 0.0f) {
					const Eigen::Vector3d bary(1 - ray.hit.u - ray.hit.v, ray.hit.u, ray.hit.v);
					uint tri_idx  = ray.hit.bvh_triangle & 0xFFFFF;
					uint blas_idx = ray.hit.bvh_triangle >> 20;
					double raw    = tlas.blas[blas_idx].surface->tri_e_MN().Evaluate(tri_idx, bary);
					pressure[i] += surface_weight[i][j0] * raw;
					t_array[j] = ray.hit.t;
				}
			}
			tactile_state_msg_.sensors[0].values[i] = pressure[i];
		}
	}
	bool weight_colors = false;
	if (visualize) {
		for (int i = 0; i < n_cols; ++i) {
			colors[i][3] = 1;
		}

		const float red[4]   = { 1, 0, 0, 1 };
		const float blue[4]  = { 0, 0, 1, 1 };
		const float green[4] = { 0, 1, 0, 1 };
		mjtNum pos[3], posS[3], tmp[3];
		mjtNum size[3]  = { 0.001, 0.001, 0.01 };
		mjtNum sizeS[3] = { 0.0005, 0.0005, 0.0005 };
		mjtNum sizeS0[3] = { 0.0001, 0.0001, 0.0001 };

		Eigen::Matrix<double, 4, Eigen::Dynamic> taxels_at_M        = M * taxel_point_mat;
		Eigen::Matrix<double, 3, Eigen::Dynamic> taxels_at_M3       = taxels_at_M.topRows<3>();
		Eigen::Matrix<double, 4, Eigen::Dynamic> taxel_normal_at_M  = M * taxel_normal_mat;
		Eigen::Matrix<double, 3, Eigen::Dynamic> taxel_normal_at_M3 = taxel_normal_at_M.topRows<3>();

		std::vector<std::vector<int>> color_idx(m);
		std::vector<std::vector<double>> color_weights(m);

		for (int i = 0; i < n; ++i) {
			pos[0] = taxels_at_M3(0, i);
			pos[1] = taxels_at_M3(1, i);
			pos[2] = taxels_at_M3(2, i);

			float scale          = std::min(std::max(pressure[i], 0.0), max_pressure) / max_pressure;
			const float color[4] = { scale, 0, 1.0f - scale, 1 };
			mjtNum s             = 0.0005 + scale * 0.002;
			mjtNum sizeP[3]      = { s, s, s };
			// initVGeom(mjGEOM_SPHERE, sizeP, pos, NULL, colors[i % n_cols]);

			if (taxel_normal_at_M3.col(i).squaredNorm() == 0) {
				initVGeom(mjGEOM_SPHERE, size, pos, NULL, colors[i % n_cols]);
			} else {
				tmp[0] = pos[0] + 0.005 * taxel_normal_at_M3(0, i);
				tmp[1] = pos[1] + 0.005 * taxel_normal_at_M3(1, i);
				tmp[2] = pos[2] + 0.005 * taxel_normal_at_M3(2, i);

				if (n_vGeom < mujoco_ros::contact_surfaces::MAX_VGEOM && visualization_mode > 1 ) {
					mjvGeom *g = vGeoms + n_vGeom++;
					mjv_initGeom(g, mjGEOM_ARROW, sizeA, pos, NULL, colors[i % n_cols]);
					mjv_connector(g, mjGEOM_ARROW, 0.0004, pos, tmp);
				}
			}

			for (int j0 = 0; j0 < surface_idx[i].size(); ++j0) {
				int j = surface_idx[i][j0];

				color_idx[j].push_back(i % n_cols);
				color_weights[j].push_back(surface_weight[i][j0]);
			}
		}
		for (int j = 0; j < m; ++j) {
			if (n_vGeom < mujoco_ros::contact_surfaces::MAX_VGEOM) {
				pos[0] = surface_at_M3(0, j);
				pos[1] = surface_at_M3(1, j);
				pos[2] = surface_at_M3(2, j);

				tmp[0]       = pos[0] + 0.005 * surface_normal_at_M3(0, j);
				tmp[1]       = pos[1] + 0.005 * surface_normal_at_M3(1, j);
				tmp[2]       = pos[2] + 0.005 * surface_normal_at_M3(2, j);
				mjvGeom *g   = vGeoms + n_vGeom++;
				float col[4] = { 0, 0, 0 };
				double w     = 0;
				double w_sum = 0;

				if (weight_colors) {
					for (int i = 0; i < color_idx[j].size(); ++i) {
						int cidx = color_idx[j][i];						
						w = color_weights[j][i];
						for (int k = 0; k < 4; ++k) {							
							col[k] += w * colors[cidx][k]; 
						}
						w_sum += w;
					}
					for (int k = 0; k < 4; ++k) {
						col[k] /= w_sum;
					}
				} else {
					for (int i = 0; i < color_idx[j].size(); ++i) {
						if (color_weights[j][i] > w) {
							w = color_weights[j][i];
							for (int k = 0; k < 4; ++k) {
								col[k] = colors[color_idx[j][i]][k];
							}
						}
					}
				}
			
				if (t_array[j] > 0 and visualization_mode > 0) {
					if (n_vGeom < mujoco_ros::contact_surfaces::MAX_VGEOM) {
						float3 normal =
						    float3(surface_normal_at_M3(0, j), surface_normal_at_M3(1, j), surface_normal_at_M3(2, j));
						float3 surface_point = float3(surface_at_M3(0, j), surface_at_M3(1, j), surface_at_M3(2, j));

						float3 hitp = -(t_array[j] + 1e-8) * 2 * normal + surface_point;

						mjtNum pos1[3] = { hitp[0], hitp[1], hitp[2] };
						mjvGeom *g     = vGeoms + n_vGeom++;
						mjv_initGeom(g, mjGEOM_ARROW, NULL, pos, NULL, col);
						mjv_connector(g, mjGEOM_ARROW, 0.0001, pos, pos1);
					}
				}

				if (true) {
					mjv_initGeom(g, mjGEOM_SPHERE, sizeS0, pos, NULL, col);
				} else {
					mjv_initGeom(g, mjGEOM_ARROW, sizeA, pos, NULL, col);
					mjv_connector(g, mjGEOM_ARROW, 0.0002, pos, tmp);
				}
			}
		}
	}
}

} // namespace mujoco_ros::contact_surfaces::sensors

PLUGINLIB_EXPORT_CLASS(mujoco_ros::contact_surfaces::sensors::CurvedSensor, mujoco_ros::contact_surfaces::SurfacePlugin)
