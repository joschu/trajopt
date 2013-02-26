#pragma once
#include "typedefs.hpp"
#include <openrave/openrave.h>

namespace trajopt {

/**
Extract trajectory array from solution vector x using indices in array vars
*/
TrajArray TRAJOPT_API getTraj(const vector<double>& x, const VarArray& vars);

inline Vector3d toVector3d(const OR::Vector& v) {
  return Vector3d(v.x, v.y, v.z);
}
inline Vector4d toVector4d(const OR::Vector& v) {
  return Vector4d(v.x, v.y, v.z, v.w);
}
Eigen::Matrix3d toRot(const OR::Vector& rq);

inline OR::Transform toRaveTransform(const Vector4d& q, const Vector3d& p) {
  return OR::Transform(OR::Vector(q[0], q[1], q[2], q[3]),
                       OR::Vector(p[0], p[1], p[2]));
}
inline DblVec trajToDblVec(const TrajArray& x) {
  return DblVec(x.data(), x.data()+x.rows()*x.cols());
}


}

