/**
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

#pragma once

#include <pluginlib/class_loader.h>

#include <mujoco_ros/mujoco_sim.h>
#include <mujoco_ros/plugin_utils.h>

#include "mujoco.h"

#include "drake/geometry/proximity/make_sphere_mesh.h"
#include "drake/geometry/proximity/make_sphere_field.h"
#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/geometry/proximity/make_box_field.h"
#include "drake/geometry/proximity/make_cylinder_mesh.h"
#include "drake/geometry/proximity/make_cylinder_field.h"
#include "drake/geometry/proximity/volume_to_surface_mesh.h"
#include "drake/geometry/proximity/polygon_surface_mesh.h"
#include "drake/geometry/proximity/obb.h"
#include "drake/geometry/proximity/field_intersection.h"
#include "drake/geometry/proximity/mesh_intersection.h"
#include "drake/geometry/proximity/mesh_plane_intersection.h"
#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "drake/geometry/proximity/bvh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/hydroelastics/hydroelastic_engine.h"
#include "drake/multibody/plant/hydroelastic_traction_calculator.h"
#include "drake/multibody/plant/coulomb_friction.h"
#include "drake/multibody/triangle_quadrature/gaussian_triangle_quadrature_rule.h"

namespace mujoco_contact_surfaces {

using namespace drake;
using namespace drake::geometry;
using namespace drake::geometry::internal;
using namespace drake::math;
using namespace drake::multibody;
using namespace drake::multibody::internal;

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
	const VolumeMeshFieldLinear<double, double> *pf;
	const Bvh<Obb, VolumeMesh<double>> *bvh_v;
	const TriangleSurfaceMesh<double> *sm;
	const Bvh<Obb, TriangleSurfaceMesh<double>> *bvh_s;
	const double hydroelastic_modulus;
	const double dissipation;

	ContactProperties(int geom_id, std::string geom_name, contactType contact_type, Shape *shape, VolumeMesh<double> *vm,
	                  VolumeMeshFieldLinear<double, double> *pf, Bvh<Obb, VolumeMesh<double>> *bvh_v,
	                  double hydroelastic_modulus, double dissipation)
	    : mujoco_geom_id(geom_id)
	    , drake_id(GeometryId::get_new_id())
	    , geom_name(geom_name)
	    , contact_type(contact_type)
	    , shape(shape)
	    , vm(vm)
	    , pf(pf)
	    , bvh_v(bvh_v)
	    , hydroelastic_modulus(hydroelastic_modulus)
	    , dissipation(dissipation){};
	ContactProperties(int geom_id, std::string geom_name, contactType contact_type, Shape *shape,
	                  TriangleSurfaceMesh<double> *sm, Bvh<Obb, TriangleSurfaceMesh<double>> *bvh_s)
	    : mujoco_geom_id(geom_id)
	    , drake_id(GeometryId::get_new_id())
	    , geom_name(geom_name)
	    , contact_type(contact_type)
	    , shape(shape)
	    , sm(sm)
	    , bvh_s(bvh_s)
	    , hydroelastic_modulus(std::numeric_limits<double>::infinity())
	    , dissipation(1.0){};
	ContactProperties(int geom_id, std::string geom_name, contactType contact_type)
	    : mujoco_geom_id(geom_id)
	    , drake_id(GeometryId::get_new_id())
	    , geom_name(geom_name)
	    , contact_type(contact_type)
	    , hydroelastic_modulus(std::numeric_limits<double>::infinity())
	    , dissipation(1.0){};
} ContactProperties;

typedef struct PointCollision
{
	Vector3<double> p;
	Vector3<double> n;
	double fn0;
	double stiffness;
	double damping;
} PointCollision;

typedef struct GeomCollision
{
	std::vector<PointCollision> pointCollisions;
	ContactSurface<double> *s;
	int g1;
	int g2;
	GeomCollision(int g1, int g2, ContactSurface<double> *s) : g1(g1), g2(g2), s(s){};
} GeomCollision;

class MujocoContactSurfacesPlugin : public MujocoSim::MujocoPlugin
{
public:
	virtual ~MujocoContactSurfacesPlugin();

	// Overlead entry point
	virtual bool load(mjModelPtr m, mjDataPtr d);

	// Called by mujoco
	virtual void update();

	// Called on reset
	virtual void reset();
	int collision_cb(const mjModel *m, const mjData *d, mjContact *con, int g1, int g2, mjtNum margin);
	void passive_cb(const mjModel *m, mjData *d);

protected:
	// Mujoco model and data pointers
	mjModelPtr m_;
	mjDataPtr d_;

private:
	// Buffer of visual geoms
	mjvGeom *vGeoms = new mjvGeom[10000];
	int n_vGeom     = 0;

	const std::string PREFIX = "cs::";

	HydroelasticContactRepresentation hydroelastic_contact_representation = HydroelasticContactRepresentation::kPolygon;
	bool visualizeContactSurfaces                                         = false;

	std::map<int, ContactProperties *> contactProperties{};
	void parseMujocoCustomFields(mjModel *m);
	void initCollisionFunction();
	std::vector<GeomCollision *> geomCollisions;
	void evaluateContactSurface(const mjModel *m, const mjData *d, GeomCollision *gc);
};

} // namespace mujoco_contact_surfaces
