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

#ifdef BENCHMARK_TACTILE
struct Timer
{
	Timer() { reset(); }
	float elapsed() const
	{
		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - start);
		return (float)time_span.count();
	}
	void reset() { start = std::chrono::high_resolution_clock::now(); }
	std::chrono::high_resolution_clock::time_point start;
};

struct Benchmark
{
public:
	void report()
	{
		double avg_build_blas, avg_build_tlas, avg_num_blas, avg_trace, avg_hits, avg_total, avg_rays, avg_tri;
		get_avgs(avg_build_blas, avg_build_tlas, avg_num_blas, avg_trace, avg_hits, avg_total, avg_rays, avg_tri);
		last_report = ros::Time::now();
		ROS_WARN_STREAM("Benchmark stats for "
		                << name << " :\n\tbuild BLAS: " << avg_build_blas << "ms\n\tbuild TLAS: " << avg_build_tlas
		                << "ms\n\tnum BLAS: " << avg_num_blas << "\n\ttrace: " << avg_trace << "ms\n\thits: " << avg_hits
		                << "\n\ttotal time: " << avg_total << "ms\n\t#rays: " << avg_rays << "\n\t#triangles: " << avg_tri
		                << "\n\tnum samples: " << num_filled << "\n");
	};

	void get_avgs(double &avg_build_blas, double &avg_build_tlas, double &avg_num_blas, double &avg_trace,
	              double &avg_hits, double &avg_total, double &avg_rays, double &avg_tri)
	{
		avg_build_blas = 0.;
		avg_build_tlas = 0.;
		avg_num_blas   = 0.;
		avg_trace      = 0.;
		avg_hits       = 0.;
		avg_total      = 0.;
		avg_rays       = 0.;
		avg_tri        = 0.;
		for (uint i = 0; i < num_filled; i++) {
			avg_build_blas += build_blas[i];
			avg_build_tlas += build_tlas[i];
			avg_num_blas += num_blas[i];
			avg_trace += trace[i];
			avg_hits += hits[i];
			avg_rays += num_rays[i];
			avg_tri += num_tri[i];
		}
		double factor = 1000.0 / num_filled;
		avg_build_blas *= factor;
		avg_build_tlas *= factor;
		avg_trace *= factor;

		if (avg_num_blas > 1) {
			avg_num_blas /= num_filled;
		}
		avg_hits /= num_filled;
		avg_rays /= num_filled;
		avg_tri /= num_filled;

		avg_total = avg_build_blas + avg_build_tlas + avg_trace;
	}

	void add_measure(double bblas, double btlas, double nblas, double tr, double h, double nrays, double ntri)
	{
		build_blas[current_idx] = bblas;
		build_tlas[current_idx] = btlas;
		num_blas[current_idx]   = nblas;
		trace[current_idx]      = tr;
		hits[current_idx]       = h;
		num_rays[current_idx]   = nrays;
		num_tri[current_idx]    = ntri;
		current_idx++;
		num_filled = std::min(num_filled + 1u, 100u);
		if (current_idx >= 100) {
			current_idx = 0;
		}
	}

	std::string name = "";
	double build_blas[100];
	double build_tlas[100];
	double num_blas[100];
	double trace[100];
	double hits[100];
	double num_rays[100];
	double num_tri[100];

	ros::Time last_report = ros::Time(0);

	uint num_filled  = 0;
	uint current_idx = 0;
};
#endif

class FlatTactileSensor : public TactileSensorBase
{
public:
	// Overloaded entry point
	virtual bool load(mjModelPtr m, mjDataPtr d) override;

#ifdef BENCHMARK_TACTILE
	Timer timer;
	Benchmark benchmark_bvh;
	Benchmark benchmark_mt;
	Benchmark benchmark_proj;
	bool skip_update     = false;
	bool print_benchmark = true;
#endif

protected:
	virtual void internal_update(const mjModel *m, mjData *d,
	                             const std::vector<GeomCollisionPtr> &geomCollisions) override;
	void projection_update(const mjModel *m, mjData *d, const std::vector<GeomCollisionPtr> &geomCollisions);
	void mt_update(const mjModel *m, mjData *d, const std::vector<GeomCollisionPtr> &geomCollisions);
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
	float sigma             = -1.0;
	int sampling_resolution = 5; // 25 samples per cell
	double resolution;
	int cx, cy;
	// color scaling factors for tactile visualization
	double tactile_running_scale = 3.;
	double tactile_current_scale = 0.;
	Eigen::Vector3d sensor_normal;

	float max_dist;
	float di_factor;
	float sub_halfwidth;
	float rmean;
	float rSampling_resolution;

	// Dynamic reconfigure
	dynamic_reconfigure::Server<mujoco_contact_surface_sensors::DynamicFlatTactileConfig> dynamic_param_server;
	void dynamicParamCallback(mujoco_contact_surface_sensors::DynamicFlatTactileConfig &config, uint32_t level,
	                          mjModelPtr m);
	std::mutex dynamic_param_mutex;
};

} // namespace mujoco_ros::contact_surfaces::sensors
