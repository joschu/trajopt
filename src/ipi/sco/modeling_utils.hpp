#pragma once
#include "modeling.hpp"
#include "num_diff.hpp"

/*
 * Utilities for creating Cost and Constraint objects given a scalar or vector-valued function
 * using analytic or numerical derivatives
 */



namespace ipi {
namespace sco {

#define DEFAULT_EPSILON 1e-5;

using Eigen::VectorXd;
using Eigen::MatrixXd;

inline vector<double> toDblVec(const VectorXd& x) {
  return vector<double>(x.data(), x.data()+x.size());
}
inline VectorXd toVectorXd(const vector<double>& x) {
  return Eigen::Map<const VectorXd>(x.data(), x.size());
}
VectorXd getVec(const vector<double>& x, const VarVector& vars);


class CostFromNumDiff : public Cost {
  ScalarOfVector f_;
  VarVector vars_;
  bool full_hessian_;
public:
  const static double epsilon = DEFAULT_EPSILON;
  CostFromNumDiff(const ScalarOfVector& f, const VarVector& vars, bool full_hessian=false);
  double value(const vector<double>& x);
  ConvexObjectivePtr convex(const vector<double>& x, Model* model);
};

class CostFromNumDiffErr : public Cost {
public:
  enum Penalty_t {
    SQUARED,
    ABS
  };
  const static double epsilon = DEFAULT_EPSILON;
  CostFromNumDiffErr(const VectorOfVector& f, const VarVector& vars, const VectorXd& coeffs, Penalty_t pen_type, const string&  name);
  double value(const vector<double>& x);
  ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  string name() {return name_;}
protected:
  VectorOfVector f_;
  VarVector vars_;
  VectorXd coeffs_;
  Penalty_t pen_type_;
  string name_;
};

class ConstraintFromNumDiff : public Constraint {
  VectorOfVector f_;
  VarVector vars_;
  Constraint::Type_t type_;
  std::string name_;
public:
  const static double epsilon = DEFAULT_EPSILON;
  ConstraintFromNumDiff(const VectorOfVector& f, const VarVector& vars, Constraint::Type_t type, const std::string& name);
  vector<double> value(const vector<double>& x);
  ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
  Type_t type() {return type_;}
  string name() {return name_;}
};


}
}
