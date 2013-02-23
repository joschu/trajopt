#pragma once
#include "modeling.hpp"
#include "num_diff.hpp"
#include "sco_common.hpp"
/**
@file modeling_utils.hpp
@brief Build problem from user-defined functions
Utilities for creating Cost and Constraint objects from functions
using numerical derivatives or user-defined analytic derivatives.


 */



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
  CostFromNumDiff(ScalarOfVectorPtr f, const VarVector& vars, bool full_hessian=false);
  double value(const vector<double>& x);
  ConvexObjectivePtr convex(const vector<double>& x, Model* model);
protected:
  ScalarOfVectorPtr f_;
  VarVector vars_;
  bool full_hessian_;
  double epsilon_;
};

class CostFromNumDiffErr : public Cost {
public:
  CostFromNumDiffErr(VectorOfVectorPtr f, const VarVector& vars, const VectorXd& coeffs, PenaltyType pen_type, const string&  name);
  double value(const vector<double>& x);
  ConvexObjectivePtr convex(const vector<double>& x, Model* model);
protected:
  VectorOfVectorPtr f_;
  VarVector vars_;
  VectorXd coeffs_;
  PenaltyType pen_type_;
  double epsilon_;
};

class ConstraintFromNumDiff : public Constraint {
public:
  ConstraintFromNumDiff(VectorOfVectorPtr f, const VarVector& vars, ConstraintType type, const std::string& name);
  vector<double> value(const vector<double>& x);
  ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
  ConstraintType type() {return type_;}
protected:
  VectorOfVectorPtr f_;
  VarVector vars_;
  ConstraintType type_;
  double epsilon_;
};


}
