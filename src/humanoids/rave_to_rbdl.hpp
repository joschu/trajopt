#pragma once
#include <rbdl.h>
#include "macros.h"
#include <openrave/openrave.h>
#include <map>

namespace trajopt {

  namespace rbd = RigidBodyDynamics;
  namespace rbdmath = RigidBodyDynamics::Math;

  TRAJOPT_API Eigen::Matrix3d toMatrix3d(const OpenRAVE::TransformMatrix& tm);

  TRAJOPT_API rbdmath::SpatialTransform toSpatialTransform(const OpenRAVE::Transform& T);

  inline OpenRAVE::Vector toRave(const Eigen::Vector3d& v) {
    return OpenRAVE::Vector(v[0], v[1], v[2]);
  }

  TRAJOPT_API OpenRAVE::Transform toRave(const rbdmath::SpatialTransform& T);

  typedef std::map<OpenRAVE::KinBody::LinkPtr, unsigned> Link2ID;
  TRAJOPT_API boost::shared_ptr<rbd::Model> MakeRBDLModel(OpenRAVE::RobotBasePtr robot, bool floating_base, Link2ID& link2id);

}
