#include "num_diff.hpp"
using namespace Eigen;
using namespace ipi::sco;

namespace ipi {
namespace sco {

VectorXd calcForwardNumGrad(const ScalarOfVector& f, const VectorXd& x, double epsilon) {
  VectorXd out(x.size());
  VectorXd xpert = x;
  double y = f(x);
  for (size_t i=0; i < size_t(x.size()); ++i) {
    xpert(i) = x(i) + epsilon;
    double ypert = f(xpert);
    out(i) = (ypert - y) / epsilon;
    xpert(i) = x(i);
  }
  return out;
}
MatrixXd calcForwardNumJac(const VectorOfVector& f, const VectorXd& x, double epsilon) {
  VectorXd y = f(x);
  MatrixXd out(y.size(), x.size());
  VectorXd xpert = x;
  for (size_t i=0; i < size_t(x.size()); ++i) {
    xpert(i) = x(i) + epsilon;
    VectorXd ypert = f(xpert);
    out.col(i) = (ypert - y) / epsilon;
    xpert(i) = x(i);
  }
  return out;
}

void calcGradAndDiagHess(const ScalarOfVector& f, const VectorXd& x,
    double epsilon, double& y, VectorXd& grad, VectorXd& hess) {
  y = f(x);
  grad.resize(x.size());
  hess.resize(x.size());
  VectorXd xpert = x;
  for (size_t i=0; i < size_t(x.size()); ++i) {
    xpert(i) = x(i) + epsilon/2;
    double yplus = f(xpert);
    xpert(i) = x(i) - epsilon/2;
    double yminus = f(xpert);
    grad(i) = (yplus - yminus)/epsilon;
    hess(i) = (yplus + yminus - 2*y) / (epsilon*epsilon/4);
    xpert(i) = x(i);
  }
}

void calcGradHess(const ScalarOfVector& f, const VectorXd& x, double epsilon,
    double& y, VectorXd& grad, MatrixXd& hess) {
  VectorOfVector grad_func = forwardNumGrad(f, epsilon);
  y = f(x);
  grad = grad_func(x);
  hess = calcForwardNumJac(grad_func, x, epsilon);
  hess = (hess + hess.transpose())/2;
}


struct ForwardNumGrad {
  ScalarOfVector f_;
  double epsilon_;
  ForwardNumGrad(const ScalarOfVector& f, double epsilon) : f_(f), epsilon_(epsilon) {}
  VectorXd operator()(const VectorXd& x) {
    return calcForwardNumGrad(f_, x, epsilon_);
  }
};

struct ForwardNumJac {
  VectorOfVector f_;
  double epsilon_;
  ForwardNumJac(const VectorOfVector& f, double epsilon) : f_(f), epsilon_(epsilon) {}
  MatrixXd operator()(const VectorXd& x) {
    return calcForwardNumJac(f_, x, epsilon_);
  }
};

VectorOfVector forwardNumGrad(const ScalarOfVector& f, double epsilon) {
  return ForwardNumGrad(f, epsilon);
}
MatrixOfVector forwardNumJac(const VectorOfVector& f, double epsilon) {
  return ForwardNumJac(f, epsilon);
}


}
}


