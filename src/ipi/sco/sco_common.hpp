#pragma once
#include <boost/foreach.hpp>
#include <vector>
#include <cmath>

namespace ipi {
namespace sco {

using std::vector;
typedef vector<double> DblVec;

inline double vecSum(const DblVec& v) {
  double out = 0;
  BOOST_FOREACH(const double& x, v) out += x;
  return out;
}
inline double vecAbsSum(const DblVec& v) {
  double out = 0;
  BOOST_FOREACH(const double& x, v) out += fabs(x);
  return out;
}
inline double pospart(double x) {
  return (x > 0) ? x : 0;
}
inline double sq(double x) {
  return x*x;
}
inline double vecHingeSum(const DblVec& v) {
  double out = 0;
  BOOST_FOREACH(const double& x, v) out += pospart(x);
  return out;
}



}
}

