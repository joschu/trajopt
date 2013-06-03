#include <openrave-core.h>
#include <openrave/openrave.h>
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "sco/modeling.hpp"
#include <boost/foreach.hpp>
#include "sco/expr_op_overloads.hpp"
using namespace Eigen;
using namespace std;

namespace trajopt {


template <typename T> // var or expr
void FiniteDifferences(const BasicArray<T>& x, AffArray& dx, double dt) {
  dx.resize(x.rows()-1, x.cols());
  for (int i=0; i < x.rows(); ++i) {
    for (int j=0; j < x.cols(); ++j) {
      dx(i,j) = x(i+1,j) - x(i,j);
    }
  }
}


OpenRAVE::Vector toRaveQuat(const Vector4d& v) {
  return OpenRAVE::Vector(v[0], v[1], v[2], v[3]);
}
OpenRAVE::Vector toRaveVector(const Vector3d& v) {
  return OpenRAVE::Vector(v[0], v[1], v[2]);
}


template<typename T>
void extend(vector<T>& a, const vector<T>& b) {
  a.insert(a.end(), b.begin(), b.end());
}

template<typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b, const vector<T>& c) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  out.insert(out.end(), c.begin(), c.end());
  return out;
}
template<typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b, const vector<T>& c, const vector<T>& d) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  out.insert(out.end(), c.begin(), c.end());
  out.insert(out.end(), d.begin(), d.end());
  return out;
}
template<typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b, const vector<T>& c, const vector<T>& d, const vector<T>& e) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  out.insert(out.end(), c.begin(), c.end());
  out.insert(out.end(), d.begin(), d.end());
  out.insert(out.end(), e.begin(), e.end());
  return out;
}




template<typename S, typename T, typename U>
vector<U> cross1(const vector<S>& a, const vector<T>& b) {
  vector<U> c(3);
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
  return c;
}

void exprInc(AffExprVector& a, const AffExprVector&  b) {
  for (int i=0; i < a.size(); ++i) {
    exprInc(a[i], b[i]);
  }
}
void exprDec(AffExprVector& a, const AffExprVector&  b) {
  for (int i=0; i < a.size(); ++i) {
    exprDec(a[i], b[i]);
  }
}
AffExprVector linearizedCrossProduct(const DblVec& x, const AffExprVector& a, const AffExprVector& b) {
  assert(a.size() == 3 && b.size() == 3);
  DblVec aval(3), bval(3);
  for (int i=0; i < 3; ++i) {
    aval[i] = a[i].value(x);
    bval[i] = b[i].value(x);
  }
  AffExprVector c(3);
  exprInc(c, cross1<AffExpr, double, AffExpr>(a, bval));
  exprInc(c, cross1<double, AffExpr, AffExpr>(aval, b));
  DblVec acrossbval = cross1<double,double,double>(aval, bval);
  for (int i=0; i<3; ++i) exprDec(c[i], acrossbval[i]);
  return c;
}



AffExprVector transformExpr(const OR::Transform& T, const AffExprVector& v) {
  OR::TransformMatrix M(T);
  AffExprVector out(3);
  for (int i=0; i < 3; ++i) {
    for (int j=0; j < 3; ++j) {
      exprInc(out[i], M.rot(i,j) * v[j]);
    }
    exprInc(out[i], T.trans[i]);
  }
  return out;
}
AffExprVector rotateExpr(const OR::Vector& rot, const AffExprVector& v) {
  OR::Transform T;
  T.rot = rot;
  return transformExpr(T, v);
}


}
