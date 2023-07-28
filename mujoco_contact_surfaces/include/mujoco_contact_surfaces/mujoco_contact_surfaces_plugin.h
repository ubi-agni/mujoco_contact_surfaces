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
 *  Copyright (c) 2023, Bielefeld University
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

#pragma once
#include <boost/shared_ptr.hpp>

#include <chrono>
#include <pluginlib/class_loader.h>

#include <mujoco_ros/mujoco_env.h>
#include <mujoco_ros/plugin_utils.h>

#include <mujoco/mujoco.h>

#include <drake/geometry/proximity/make_sphere_mesh.h>
#include <drake/geometry/proximity/make_sphere_field.h>
#include <drake/geometry/proximity/make_box_mesh.h>
#include <drake/geometry/proximity/make_box_field.h>
#include <drake/geometry/proximity/make_cylinder_mesh.h>
#include <drake/geometry/proximity/make_cylinder_field.h>
#include <drake/geometry/proximity/make_convex_field.h>
#include <drake/geometry/proximity/make_convex_mesh.h>
#include <drake/geometry/proximity/volume_to_surface_mesh.h>
#include <drake/geometry/proximity/polygon_surface_mesh.h>
#include <drake/geometry/proximity/triangle_surface_mesh.h>
#include <drake/geometry/proximity/obb.h>
#include <drake/geometry/proximity/field_intersection.h>
#include <drake/geometry/proximity/mesh_intersection.h>
#include <drake/geometry/proximity/mesh_plane_intersection.h>
#include <drake/geometry/proximity/obj_to_surface_mesh.h>
#include <drake/geometry/proximity/bvh.h>
#include <drake/geometry/geometry_ids.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/hydroelastics/hydroelastic_engine.h>
#include <drake/multibody/plant/hydroelastic_traction_calculator.h>
#include <drake/multibody/plant/coulomb_friction.h>
#include <drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h>
#include <pluginlib/class_loader.h>
#include <mujoco_contact_surfaces/common_types.h>
#include <mujoco_contact_surfaces/plugin_utils.h>

namespace mujoco_ros::contact_surfaces {

using namespace std::chrono;
using namespace drake;
using namespace drake::geometry;
using namespace drake::geometry::internal;
using namespace drake::math;
using namespace drake::multibody;
using namespace drake::multibody::internal;
using namespace mujoco_ros;
using namespace std::chrono;

const std::string PREFIX = "cs::";

typedef enum _contactType
{
	RIGID,
	SOFT
} contactType;

// Properties of geoms used for computing contact surfaces
typedef struct ContactProperties
{
	const int mujoco_geom_id;
	GeometryId drake_id;
	const std::string geom_name;
	const contactType contact_type;
	const Shape *shape;
	const VolumeMesh<double> *vm;
	std::shared_ptr<const VolumeMeshFieldLinear<double, double>> pf;
	std::shared_ptr<const Bvh<Obb, VolumeMesh<double>>> bvh_v;
	const TriangleSurfaceMesh<double> *sm;
	std::shared_ptr<const Bvh<Obb, TriangleSurfaceMesh<double>>> bvh_s;
	const double hydroelastic_modulus;
	const double dissipation;
	const double static_friction;
	const double dynamic_friction;
	const double resolution_hint;

	~ContactProperties() {
		if (shape != nullptr) delete shape;
		// if (pf != nullptr) delete pf;
		// if (bvh_v != nullptr) delete bvh_v;
		// if (vm != nullptr) delete vm; // bvh_v owns vm
		// if (sm != nullptr) delete sm;
		// if (bvh_s != nullptr) delete bvh_s;

	}

	ContactProperties(int geom_id, std::string geom_name, contactType contact_type, Shape *shape, VolumeMesh<double> *vm,
	                  VolumeMeshFieldLinear<double, double> *pf, Bvh<Obb, VolumeMesh<double>> *bvh_v,
	                  double hydroelastic_modulus, double dissipation, double static_friction, double dynamic_friction,
	                  double resolution_hint)
	    : mujoco_geom_id(geom_id)
	    , drake_id(GeometryId::get_new_id())
	    , geom_name(geom_name)
	    , contact_type(contact_type)
	    , shape(shape)
	    , vm(vm)
	    , pf(pf)
	    , bvh_v(bvh_v)
	    , hydroelastic_modulus(hydroelastic_modulus)
	    , dissipation(dissipation)
	    , static_friction(static_friction)
	    , dynamic_friction(dynamic_friction)
	    , resolution_hint(resolution_hint){};
	ContactProperties(int geom_id, std::string geom_name, contactType contact_type, Shape *shape,
	                  TriangleSurfaceMesh<double> *sm, Bvh<Obb, TriangleSurfaceMesh<double>> *bvh_s,
	                  double static_friction, double dynamic_friction, double resolution_hint)
	    : mujoco_geom_id(geom_id)
	    , drake_id(GeometryId::get_new_id())
	    , geom_name(geom_name)
	    , contact_type(contact_type)
	    , shape(shape)
	    , sm(sm)
	    , bvh_s(bvh_s)
	    , hydroelastic_modulus(std::numeric_limits<double>::infinity())
	    , dissipation(1.0)
	    , static_friction(static_friction)
	    , dynamic_friction(dynamic_friction)
	    , resolution_hint(resolution_hint){};
	ContactProperties(int geom_id, std::string geom_name, contactType contact_type, double static_friction,
	                  double dynamic_friction)
	    : mujoco_geom_id(geom_id)
	    , drake_id(GeometryId::get_new_id())
	    , geom_name(geom_name)
	    , contact_type(contact_type)
	    , hydroelastic_modulus(std::numeric_limits<double>::infinity())
	    , dissipation(1.0)
	    , static_friction(static_friction)
	    , dynamic_friction(dynamic_friction)
	    , resolution_hint(0.0){};
} ContactProperties;

class MujocoContactSurfacesPlugin : public mujoco_ros::MujocoPlugin
{
public:
	virtual ~MujocoContactSurfacesPlugin();

	// Overloaded entry point
	virtual bool load(mjModelPtr m, mjDataPtr d) override;

	// Called on reset
	virtual void reset() override;

	virtual void passiveCallback(mjModelPtr model, mjDataPtr data) override;
	virtual void renderCallback(mjModelPtr model, mjDataPtr data, mjvScene *scene) override;
	virtual void onGeomChanged(mjModelPtr model, mjDataPtr data, const int geom_id) override;

	void passive_cb(const mjModel *m, mjData *d);
	int collision_cb(const mjModel *m, const mjData *d, mjContact *con, int g1, int g2, mjtNum margin);

	std::vector<SurfacePluginPtr> getPlugins();

protected:
	// Mujoco model and data pointers
	mjModelPtr m_;
	mjDataPtr d_;
	bool visualizeContactSurfaces = false;
	bool applyContactSurfaceForces = true;
	std::vector<GeomCollisionPtr> geomCollisions;

private:
	// Buffer of visual geoms
	mjvGeom *vGeoms = new mjvGeom[MAX_VGEOM];
	int n_vGeom     = 0;
	// color scaling factors for contact
	double running_scale = 3.;
	double current_scale = 0.;

	// TODO there seems to be a bug where this is not correctly parsed
	HydroelasticContactRepresentation hydroelastic_contact_representation = HydroelasticContactRepresentation::kTriangle;

	std::map<int, std::shared_ptr<ContactProperties>> contactProperties;

	void parseMujocoCustomFields(mjModel *m);
	void initCollisionFunction();

	void evaluateContactSurface(const mjModel *m, const mjData *d, GeomCollisionPtr gc);
	template <class T>
	void visualizeMeshElement(int face, T mesh, double fn);

	// Interface loader
	boost::shared_ptr<pluginlib::ClassLoader<SurfacePlugin>> surface_plugin_loader;
	// list of registered and loaded plugins
	std::vector<SurfacePluginPtr> plugins, cb_ready_plugins;
};

} // namespace mujoco_ros::contact_surfaces
