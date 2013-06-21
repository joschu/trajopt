#pragma once
#include <Eigen/Dense>
using namespace Eigen;


inline Vector4d quatMult(const Vector4d& q1, const Vector4d& q2) {
  return Vector4d(
      q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
      q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
      q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3],
      q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1]);
}

inline Vector4d quatExp(const Vector3d& r) {
  // see http://www.lce.hut.fi/~ssarkka/pub/quat.pdf
  double normr = r.norm();
  if (normr > 1e-10) {
    Vector4d q;
    q(0) = cos(normr / 2);
    q.bottomRows(3) = (r/normr) * sin(normr/2);
    return q;
  }
  else {
    Vector4d out(1,0,0,0);
    out.bottomRows(3) += r;
    out.normalize();
    return out;
  }
}

inline Vector3d quatLog(const Vector4d& q) {
  if (1-q[0] >= 1e-10) {
    Vector3d v = q.bottomRows(3);
    double s = q(0);
    Vector3d out = (acos(s) / v.norm()) * v;
    return out;
  }
  else {
    return 2*q.bottomRows(3);
  }
}

inline Vector4d quatInv(const Vector4d& q) {
  Vector4d qinv = q;
  qinv.bottomRows(3) *= -1;
  return qinv;
}


inline VectorXd quatRotate(const Vector4d& q, const Vector3d& p) {
  Vector4d pquat;
  pquat(0) = 0;
  pquat.bottomRows(3) = p;
  return quatMult(q, quatMult(pquat, quatInv(q)));
}
