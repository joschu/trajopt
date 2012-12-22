#pragma once

/*
 * Model a non-convex optimization problem by defining Cost and Constraint objects
 * which know how to generate a convex approximation
 *
 *
 */

#include <vector>
#include <boost/shared_ptr.hpp>
#include "sco_fwd.hpp"
#include "solver_interface.hpp"

namespace ipi {
namespace sco {

using std::vector;

class ConvexObjective {
public:
  ConvexObjective(Model* model) : model_(model) {}
  void addAffExpr(const AffExpr&);
  void addQuadExpr(const QuadExpr&);
  void addHinge(const AffExpr&, double coeff);
  void addAbs(const AffExpr&, double coeff);
  void addHinges(const AffExprVector&);
  void addL1Norm(const AffExprVector&);
  void addL2Norm(const AffExprVector&);
  void addMax(const AffExprVector&);
  
  void setModel(Model* model) {
    assert(!inModel());
    model_ = model;
  }
  bool inModel() {
    return model_ != NULL;
  }
  void addConstraintsToModel();
  void removeFromModel();
  double value(); // at current values of variables in model
  // todo: should be able to evaluate at some values x
  double value(const vector<double>& x);
  
  ~ConvexObjective();

  
  Model* model_;
  QuadExpr quad_;
  vector<Var> vars_;
  vector<AffExpr> eqs_;
  vector<AffExpr> ineqs_;
  vector<Cnt> cnts_;
private:
  ConvexObjective()  {}
  ConvexObjective(ConvexObjective&)  {}
};

class ConvexConstraints {
public:
  ConvexConstraints(Model* model) : model_(model) {}
  void addEqCnt(const AffExpr&);
  void addIneqCnt(const AffExpr&);
  void setModel(Model* model) {
    assert(!inModel());
    model_ = model;
  }
  bool inModel() {
    return model_ != NULL;
  }
  void addConstraintsToModel();
  void removeFromModel();

  vector<double> violations();
  double violation();

  vector<double> violations(const vector<double>& x);
  double violation(const vector<double>& x);

  ~ConvexConstraints();
  vector<AffExpr> eqs_;
  vector<AffExpr> ineqs_;
private:
   Model* model_;
   vector<Cnt> cnts_;
   ConvexConstraints() : model_(NULL) {}
   ConvexConstraints(ConvexConstraints&) {}
};

class Cost {
public:
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model) = 0;
  virtual double value(const vector<double>&) = 0;
  virtual ~Cost() {}
  virtual string name() {return "Unnamed";}
};

class Constraint {
public:
  enum Type_t {
    EQ,
    INEQ
  };
  virtual Type_t type() = 0;
  virtual vector<double> value(const vector<double>& x) = 0;
  virtual ConvexConstraintsPtr convex(const vector<double>& x, Model* model) = 0;
  virtual string name() {return "Unnamed";}
  virtual ~Constraint() {}

  vector<double> violations(const vector<double>& x);
  double violation(const vector<double>& x);

};

class EqConstraint : public Constraint{
public:
  Type_t type() {return EQ;}
};

class IneqConstraint : public Constraint {
public:
  Type_t type() {return INEQ;}
};

class OptProb {
public:
  OptProb();
  void createVariables(const vector<string>& names);
  void createVariables(const vector<string>& names, const vector<double>& lb, const vector<double>& ub);
  void setLowerBounds(const vector<double>& lb);
  void setUpperBounds(const vector<double>& ub);

  void addCost(CostPtr);
  void addConstr(ConstraintPtr);
  void addEqConstr(ConstraintPtr);
  void addIneqConstr(ConstraintPtr);
  virtual ~OptProb() {}

  vector<ConstraintPtr> getConstraints() const;

  ModelPtr model_;
  vector<Var> vars_;
  vector<double> lower_bounds_;
  vector<double> upper_bounds_;
  vector<CostPtr> costs_;
  vector<ConstraintPtr> eqcnts_;
  vector<ConstraintPtr> ineqcnts_;
private:
  OptProb(OptProb&);
};

}}
