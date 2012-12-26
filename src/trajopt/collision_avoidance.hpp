#pragma once
#include "trajopt/common.hpp"
#include "trajopt/collision_checker.hpp"
#include "ipi/sco/modeling.hpp"
#include "ipi/sco/sco_fwd.hpp"

namespace trajopt {



/**
Collision cost and constraint use this class
*/
class SingleTimestepCollisionEvaluator {
public:
  SingleTimestepCollisionEvaluator(RobotAndDOFPtr rad, const VarVector& vars);
  /**
  @brief linearize all contact distances in terms of robot dofs
  
  Do a collision check between robot and enironment.
  For each contact generated, return a linearization of the signed distance function
  Since the collision checker may return multiple contact points for a given pair of links,
  the contacts are associated with weights so that each pair of links are associated with a single cost term.
  In particular, if a pair of bodies have k contacts, then the contacts each have weight 1/k.
  */
  void CalcDistExpressions(vector<AffExpr>& exprs, DblVec& weights); // appends to this vector
private:
  // parameters:
  double m_coeff;
  double m_safe_dist;
  EnvironmentBasePtr m_env;
  CollisionCheckerPtr m_cc;
  RobotAndDOFPtr m_rad;
  sco::VarVector m_vars;
};

class CollisionCost : public Cost {
public:
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model) = 0;
  virtual double value(const vector<double>&) = 0;
  virtual string name() {return "Collision";}
};
class CollisionCost : public Cost {
public:
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model) = 0;
  virtual double value(const vector<double>&) = 0;
  virtual string name() {return "Collision";}
};