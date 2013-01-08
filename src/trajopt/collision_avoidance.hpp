#pragma once
#include "trajopt/common.hpp"
#include "trajopt/collision_checker.hpp"
#include "ipi/sco/modeling.hpp"
#include "ipi/sco/sco_fwd.hpp"
#include "cache.hpp"


namespace trajopt {



/**
Collision cost and constraint use this class
*/
struct SingleTimestepCollisionEvaluator {
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
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs, DblVec& weights); // appends to this vector
  /**
   * Same as CalcDistExpressions, but just the distances--not the expressions
   */
  void CalcDists(const DblVec& x, DblVec& exprs, DblVec& weights); // appends to this vector

  // parameters:
  OR::EnvironmentBasePtr m_env;
  CollisionCheckerPtr m_cc;
  RobotAndDOFPtr m_rad;
  VarVector m_vars;
  CollisionPairIgnorer m_ignorer;
  typedef std::map<const OR::KinBody::Link*, int> Link2Int;
  Link2Int m_link2ind;
  vector<OR::KinBody::LinkPtr> m_links;
  Cache<double, vector<Collision>, 3> m_cache;

  void GetCollisionsCached(const DblVec& x, vector<Collision>&);
};

class CollisionCost : public Cost, public Plotter {
public:
  CollisionCost(double dist_pen, double coeff, RobotAndDOFPtr rad, const VarVector& vars);
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  virtual double value(const vector<double>&);
  virtual void Plot(const DblVec& x, OR::EnvironmentBase&, std::vector<OR::GraphHandlePtr>& handles);
private:
  SingleTimestepCollisionEvaluator m_calc;
  double m_dist_pen;
  double m_coeff;
};
}
