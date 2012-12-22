#include "modeling.hpp"
#include "ipi/logging.hpp"
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <cstdio>
#include "expr_ops.hpp"
#include "sco_common.hpp"

using namespace std;

namespace ipi {
namespace sco {

void ConvexObjective::addAffExpr(const AffExpr& affexpr) {
  exprInc(quad_, affexpr);
}
void ConvexObjective::addQuadExpr(const QuadExpr& quadexpr) {
  exprInc(quad_, quadexpr);
}
void ConvexObjective::addHinge(const AffExpr& affexpr, double coeff) {
  Var hinge = model_->addVar("", 0, INFINITY);
  vars_.push_back(hinge);
  ineqs_.push_back(affexpr);
  exprDec(ineqs_.back(), hinge);
  AffExpr hinge_cost(hinge);
  exprInc(quad_, hinge_cost);
}
void ConvexObjective::addAbs(const AffExpr& affexpr, double coeff) {
  Var neg = model_->addVar("", 0, INFINITY);
  Var pos = model_->addVar("", 0, INFINITY);
  vars_.push_back(neg);
  vars_.push_back(pos);
  AffExpr neg_plus_pos;
  neg_plus_pos.coeffs = vector<double>(2, coeff);
  neg_plus_pos.vars.push_back(neg);
  neg_plus_pos.vars.push_back(pos);
  exprInc(quad_, neg_plus_pos);
  AffExpr affeq = affexpr;
  affeq.vars.push_back(neg);
  affeq.vars.push_back(pos);
  affeq.coeffs.push_back(1);
  affeq.coeffs.push_back(-1);
  eqs_.push_back(affeq);
}
void ConvexObjective::addHinges(const AffExprVector& ev) {
  for (size_t i=0; i < ev.size(); ++i) addHinge(ev[i],1);
}
void ConvexObjective::addL1Norm(const AffExprVector& ev) {
  for (size_t i=0; i < ev.size(); ++i) addAbs(ev[i],1);
}
void ConvexObjective::addL2Norm(const AffExprVector& ev) {
  for (size_t i=0; i < ev.size(); ++i) exprInc(quad_, exprSquare(ev[i]));
}
void ConvexObjective::addMax(const AffExprVector& ev) {
  Var m = model_->addVar("m", -INFINITY, INFINITY);
  for (size_t i=0; i < ev.size(); ++i) {
    ineqs_.push_back(ev[i]);
    exprDec(ineqs_.back(), m);
  }
}

void ConvexObjective::addConstraintsToModel() {
  cnts_.reserve(eqs_.size() + ineqs_.size());
  BOOST_FOREACH(const AffExpr& aff, eqs_) {
    cnts_.push_back(model_->addEqCnt(aff, ""));
  }
  BOOST_FOREACH(const AffExpr& aff, ineqs_) {
    cnts_.push_back(model_->addIneqCnt(aff, ""));
  }
}

void ConvexObjective::removeFromModel() {
  model_->removeCnts(cnts_);
  model_->removeVars(vars_);
  model_ = NULL;
}
ConvexObjective::~ConvexObjective() {
  if (inModel()) removeFromModel();
}

void ConvexConstraints::addEqCnt(const AffExpr& aff) {
  eqs_.push_back(aff);
}

void ConvexConstraints::addIneqCnt(const AffExpr& aff) {
  ineqs_.push_back(aff);
}

void ConvexConstraints::addConstraintsToModel() {
  cnts_.reserve(eqs_.size() + ineqs_.size());
  BOOST_FOREACH(const AffExpr& aff, eqs_) {
    cnts_.push_back(model_->addEqCnt(aff, ""));
  }
  BOOST_FOREACH(const AffExpr& aff, ineqs_) {
    cnts_.push_back(model_->addIneqCnt(aff, ""));
  }
}

void ConvexConstraints::removeFromModel() {
  model_->removeCnts(cnts_);
  model_ = NULL;
}

vector<double> ConvexConstraints::violations() {
  DblVec out;
  out.reserve(eqs_.size() + ineqs_.size());
  BOOST_FOREACH(const AffExpr& aff, eqs_) out.push_back(fabs(aff.value()));
  BOOST_FOREACH(const AffExpr& aff, ineqs_) out.push_back(pospart(aff.value()));
  return out;
}
double ConvexConstraints::violation() {
  return vecSum(violations());
}
vector<double> ConvexConstraints::violations(const vector<double>& x) {
  DblVec out;
  out.reserve(eqs_.size() + ineqs_.size());
  BOOST_FOREACH(const AffExpr& aff, eqs_) out.push_back(fabs(aff.value(x.data())));
  BOOST_FOREACH(const AffExpr& aff, ineqs_) out.push_back(pospart(aff.value(x.data())));
  return out;
}
double ConvexConstraints::violation(const vector<double>& x) {
  return vecSum(violations(x));
}

ConvexConstraints::~ConvexConstraints() {
  if (inModel()) removeFromModel();
}
double ConvexObjective::value() {
  return quad_.value();
}
double ConvexObjective::value(const vector<double>& x)  {
  return quad_.value(x);
}


vector<double> Constraint::violations(const DblVec& x) {
  DblVec val = value(x);
  DblVec out(val.size());

  if (type() == EQ) {
    for (size_t i=0; i < val.size(); ++i) out[i] = fabs(val[i]);
  }
  else { // type() == INEQ
    for (size_t i=0; i < val.size(); ++i) out[i] = pospart(val[i]);
  }

  return out;
}

double Constraint::violation(const DblVec& x) {
  return vecSum(violations(x));
}

OptProb::OptProb() : model_(createModel(SOLVER_GUROBI)) {}

void OptProb::createVariables(const vector<string>& var_names) {
  createVariables(var_names, DblVec(var_names.size(), -INFINITY), DblVec(var_names.size(), INFINITY));
}
void OptProb::createVariables(const vector<string>& var_names, const DblVec& lb, const DblVec& ub) {
  size_t n_add = var_names.size(), n_cur = vars_.size();
  assert(lb.size() == n_add);
  assert(ub.size() == n_add);
  vars_.reserve(n_cur + n_add);
  lower_bounds_.reserve(n_cur + n_add);
  upper_bounds_.reserve(n_cur + n_add);
  for (size_t i=0; i < var_names.size(); ++i) {
    vars_.push_back(model_->addVar(var_names[i], lb[i], ub[i]));
    lower_bounds_.push_back(lb[i]);
    upper_bounds_.push_back(ub[i]);
  }
  model_->update();
}
void OptProb::setLowerBounds(const vector<double>& lb) {assert(lb.size() == vars_.size()); lower_bounds_ = lb;}
void OptProb::setUpperBounds(const vector<double>& ub) {assert(ub.size() == vars_.size()); upper_bounds_ = ub;}

void OptProb::addCost(CostPtr cost) {
  costs_.push_back(cost);
}
void OptProb::addConstr(ConstraintPtr cnt) {
  if (cnt->type() == Constraint::EQ) addEqConstr(cnt);
  else addIneqConstr(cnt);
}
void OptProb::addEqConstr(ConstraintPtr cnt) {
  assert (cnt->type() == Constraint::EQ);
  eqcnts_.push_back(cnt);
}
void OptProb::addIneqConstr(ConstraintPtr cnt) {
  assert (cnt->type() == Constraint::INEQ);
  ineqcnts_.push_back(cnt);
}
vector<ConstraintPtr> OptProb::getConstraints() const {
  vector<ConstraintPtr> out;
  out.reserve(eqcnts_.size() + ineqcnts_.size());
  out.insert(out.end(), eqcnts_.begin(), eqcnts_.end());
  out.insert(out.end(), ineqcnts_.begin(), ineqcnts_.end());
  return out;
}

}
}
