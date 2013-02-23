#include "utils.hpp"
#include "sco/solver_interface.hpp"
#include <Eigen/Geometry>
namespace trajopt {
TrajArray getTraj(const vector<double>& x, const VarArray& vars) {
  TrajArray out(vars.rows(), vars.cols());
  for (size_t i=0; i < (size_t)vars.rows(); ++i) {
    for (size_t j=0; j < (size_t)vars.cols(); ++j) {
      out(i,j) = vars(i,j).value(x.data());
    }
  }
  return out;
}

Eigen::Matrix3d toRot(const OR::Vector& rq) {
  Eigen::Affine3d T;
  T = Eigen::Quaterniond(rq[0], rq[1], rq[2], rq[3]);
  return T.rotation();
}



}
