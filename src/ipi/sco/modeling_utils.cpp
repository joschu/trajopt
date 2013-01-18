#include "modeling.hpp"
#include "modeling_utils.hpp"
#include "expr_ops.hpp"
#include <iostream>
#include <Eigen/Eigenvalues>
#include "utils/eigen_conversions.hpp"
using namespace std;
using namespace util;


namespace ipi {
namespace sco {

const double DEFAULT_EPSILON = 1e-5;


VectorXd getVec(const vector<double>& x, const VarVector& vars) {
  VectorXd out(vars.size());
  for (unsigned i=0; i < vars.size(); ++i) out[i] = x[vars[i].var_rep->index];
  return out;
}

DblVec getDblVec(const vector<double>& x, const VarVector& vars) {
  DblVec out(vars.size());
  for (unsigned i=0; i < vars.size(); ++i) out[i] = x[vars[i].var_rep->index];
  return out;
}

CostFromNumDiff::CostFromNumDiff(ScalarOfVectorPtr f, const VarVector& vars, bool full_hessian) :
    f_(f), vars_(vars), full_hessian_(full_hessian), epsilon_(DEFAULT_EPSILON) {}

double CostFromNumDiff::value(const vector<double>& xin) {
  VectorXd x = getVec(xin, vars_);
  return f_->call(x);
}

ConvexObjectivePtr CostFromNumDiff::convex(const vector<double>& xin, Model* model) {
  VectorXd x = getVec(xin, vars_);

  ConvexObjectivePtr out(new ConvexObjective(model));
  if (!full_hessian_) {
    double val;
    VectorXd grad,hess;
    calcGradAndDiagHess(*f_, x, epsilon_, val, grad, hess);
    hess = hess.cwiseMax(VectorXd::Zero(hess.size()));
    QuadExpr& quad = out->quad_;
    quad.affexpr.constant = val - grad.dot(x) + .5*x.dot(hess.cwiseProduct(x));
    quad.affexpr.vars = vars_;
    quad.affexpr.coeffs = toDblVec(grad - hess.cwiseProduct(x));
    quad.vars1 = vars_;
    quad.vars2 = vars_;
    quad.coeffs = toDblVec(hess*.5);
  }
  else {
    double val;
    VectorXd grad;
    MatrixXd hess;
    calcGradHess(f_, x, epsilon_, val, grad, hess);

    MatrixXd pos_hess = MatrixXd::Zero(x.size(), x.size());
    Eigen::SelfAdjointEigenSolver<MatrixXd> es(hess);
    VectorXd eigvals = es.eigenvalues();
    MatrixXd eigvecs = es.eigenvectors();
    for (size_t i=0, end = x.size(); i != end; ++i) { //tricky --- eigen size() is signed
      if (eigvals(i) > 0) pos_hess += eigvals(i) * eigvecs.col(i) * eigvecs.col(i).transpose();
    }

    QuadExpr& quad = out->quad_;
    quad.affexpr.constant = val - grad.dot(x) + .5*x.dot(pos_hess * x);
    quad.affexpr.vars = vars_;
    quad.affexpr.coeffs = toDblVec(grad - pos_hess * x);

    int nquadterms = (x.size() * (x.size()-1))/2;
    quad.coeffs.reserve(nquadterms);
    quad.vars1.reserve(nquadterms);
    quad.vars2.reserve(nquadterms);
    for (size_t i=0, end = x.size(); i != end; ++i) { //tricky --- eigen size() is signed
      quad.vars1.push_back(vars_[i]);
      quad.vars2.push_back(vars_[i]);
      quad.coeffs.push_back(pos_hess(i,i)/2);
      for (size_t j=i+1; j != end; ++j) {  //tricky --- eigen size() is signed
        quad.vars1.push_back(vars_[i]);
        quad.vars2.push_back(vars_[j]);
        quad.coeffs.push_back(pos_hess(i,j));
      }
    }
  }

  return out;
}

CostFromNumDiffErr::CostFromNumDiffErr(VectorOfVectorPtr f, const VarVector& vars, const VectorXd& coeffs, PenaltyType pen_type, const std::string& name) :
    Cost(name), f_(f), vars_(vars), coeffs_(coeffs), pen_type_(pen_type), epsilon_(DEFAULT_EPSILON) {}
double CostFromNumDiffErr::value(const vector<double>& xin) {
  VectorXd x = getVec(xin, vars_);
  VectorXd scaled_err = f_->call(x).cwiseProduct(coeffs_);
  return (pen_type_ == SQUARED) ? scaled_err.squaredNorm() : scaled_err.lpNorm<1>();
}
ConvexObjectivePtr CostFromNumDiffErr::convex(const vector<double>& xin, Model* model) {
  VectorXd x = getVec(xin, vars_);
  MatrixXd jac = calcForwardNumJac(*f_, x, epsilon_);
  ConvexObjectivePtr out(new ConvexObjective(model));
  VectorXd y = f_->call(x);
  for (int i=0; i < jac.rows(); ++i) {
    if (coeffs_[i] > 0) {
      AffExpr aff;
      aff.constant = y[i] - jac.row(i).dot(x);
      aff.coeffs = toDblVec(jac.row(i));
      aff.vars = vars_;
      aff = cleanupAff(aff);
      if (pen_type_ == SQUARED) {
        out->addQuadExpr(exprMult(exprSquare(aff), coeffs_[i]));
      }
      else {
        out->addAbs(aff, coeffs_[i]);
      }
    }
  }
  return out;
}


ConstraintFromNumDiff::ConstraintFromNumDiff(VectorOfVectorPtr f, const VarVector& vars, ConstraintType type, const std::string& name, const BoolVec& enabled) :
    Constraint(name), f_(f), vars_(vars), type_(type), epsilon_(DEFAULT_EPSILON), enabled_(enabled) {}
vector<double> ConstraintFromNumDiff::value(const vector<double>& xin) {
  VectorXd x = getVec(xin, vars_);
  return toDblVec(f_->call(x));
}
ConvexConstraintsPtr ConstraintFromNumDiff::convex(const vector<double>& xin, Model* model) {
  VectorXd x = getVec(xin, vars_);
  MatrixXd jac = calcForwardNumJac(*f_, x, epsilon_);
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  VectorXd y = f_->call(x);
  for (int i=0; i < jac.rows(); ++i) {
    if (enabled_.empty() || enabled_[i]) {
      AffExpr aff;
      aff.constant = y[i] - jac.row(i).dot(x);
      aff.coeffs = toDblVec(jac.row(i));
      aff.vars = vars_;
      aff = cleanupAff(aff);
      if (type() == INEQ) out->addIneqCnt(aff);
      else out->addEqCnt(aff);
    }
  }
  return out;
}



}
}
