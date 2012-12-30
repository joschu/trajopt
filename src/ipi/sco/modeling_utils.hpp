#pragma once
#include "modeling.hpp"
#include "num_diff.hpp"

/**
@file modeling_utils.hpp
@brief Build problem from user-defined functions
Utilities for creating Cost and Constraint objects from functions
using numerical derivatives or user-defined analytic derivatives.


 */



namespace ipi {
namespace sco {

enum PenaltyType {
  SQUARED,
  ABS
};

using Eigen::VectorXd;
using Eigen::MatrixXd;

VectorXd getVec(const vector<double>& x, const VarVector& vars);
DblVec getDblVec(const vector<double>& x, const VarVector& vars);


class CostFromNumDiff : public Cost {
public:
  CostFromNumDiff(const ScalarOfVector& f, const VarVector& vars, bool full_hessian=false);
  double value(const vector<double>& x);
  ConvexObjectivePtr convex(const vector<double>& x, Model* model);
private:
  ScalarOfVector f_;
  VarVector vars_;
  bool full_hessian_;
  double epsilon_;
};

class CostFromNumDiffErr : public Cost {
public:
  CostFromNumDiffErr(const VectorOfVector& f, const VarVector& vars, const VectorXd& coeffs, PenaltyType pen_type, const string&  name);
  double value(const vector<double>& x);
  ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  string name() {return name_;}
protected:
  VectorOfVector f_;
  VarVector vars_;
  VectorXd coeffs_;
  PenaltyType pen_type_;
  string name_;
  double epsilon_;
};

class ConstraintFromNumDiff : public Constraint {
public:
  ConstraintFromNumDiff(const VectorOfVector& f, const VarVector& vars, ConstraintType type, const std::string& name);
  vector<double> value(const vector<double>& x);
  ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
  ConstraintType type() {return type_;}
  string name() {return name_;}
private:
  VectorOfVector f_;
  VarVector vars_;
  ConstraintType type_;
  std::string name_;
  double epsilon_;
};


}
}
