#include "trajopt/collision_avoidance.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "ipi/sco/expr_vec_ops.hpp"
#include "ipi/sco/expr_ops.hpp"
#include "ipi/sco/sco_common.hpp"
#include <boost/foreach.hpp>
#include "utils/eigen_conversions.hpp"
#include "ipi/sco/modeling_utils.hpp"
using namespace OpenRAVE;
using namespace ipi::sco;
using namespace util;
using namespace std;

namespace trajopt {

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(RobotAndDOFPtr rad, const VarVector& vars) :
  m_env(rad->GetRobot()->GetEnv()),
  m_cc(CollisionChecker::GetOrCreate(*m_env)),
  m_rad(rad),
  m_vars(vars),
  m_link2ind(),
  m_links() {
  RobotBasePtr robot = rad->GetRobot();
  const vector<KinBody::LinkPtr>& robot_links = robot->GetLinks();
  vector<KinBody::LinkPtr> links;
  vector<int> inds;
  rad->GetAffectedLinks(m_links, true, inds);
  for (int i=0; i < m_links.size(); ++i) {
    m_link2ind[m_links[i].get()] = inds[i];
  }

}

#if 0

#endif

void SingleTimestepCollisionEvaluator::GetCollisionsCached(const DblVec& x, vector<Collision>& collisions) {
  double key = vecSum(x);
  vector<Collision>* it = m_cache.get(key);
  if (it != NULL) {
    collisions = *it;
  }
  else {
    m_cc->LinksVsAll(m_links, collisions);
    m_cache.put(key, collisions);
  }
}


void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists, DblVec& weights) {
  DblVec dofvals = getDblVec(x, m_vars);
  m_rad->SetDOFValues(dofvals);
  vector<Collision> collisions;
  GetCollisionsCached(dofvals, collisions);
  dists.reserve(dists.size() + collisions.size());
  weights.reserve(weights.size() + collisions.size());
  BOOST_FOREACH(const Collision& col, collisions) {
    Link2Int::iterator itA = m_link2ind.find(col.linkA),
                       itB = m_link2ind.find(col.linkB);
    if (itA != m_link2ind.end() || itB != m_link2ind.end()) {
      dists.push_back(col.distance);
      weights.push_back(col.weight);
    }
  }
}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs, DblVec& weights) {
  DblVec dofvals = getDblVec(x, m_vars);
  m_rad->SetDOFValues(dofvals);
  vector<Collision> collisions;
  GetCollisionsCached(dofvals, collisions);
//  m_cc->BodyVsAll(*m_rad->GetRobot(), &m_ignorer, collisions);
  exprs.reserve(exprs.size() + collisions.size());
  weights.reserve(weights.size() + collisions.size());

  BOOST_FOREACH(const Collision& col, collisions) {
    AffExpr dist(col.distance);
    Link2Int::iterator itA = m_link2ind.find(col.linkA);
    if (itA != m_link2ind.end()) {
      VectorXd dist_grad = toVector3d(col.normalB2A).transpose()*m_rad->PositionJacobian(itA->second, col.ptA);
      exprInc(dist, varDot(dist_grad, m_vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
    }
    Link2Int::iterator itB = m_link2ind.find(col.linkB);
    if (itB != m_link2ind.end()) {
      VectorXd dist_grad = -toVector3d(col.normalB2A).transpose()*m_rad->PositionJacobian(itB->second, col.ptB);
      exprInc(dist, varDot(dist_grad, m_vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
    }
    if (itA != m_link2ind.end() || itB != m_link2ind.end()) {
      exprs.push_back(dist);
      weights.push_back(col.weight);
    }
  }
  RAVELOG_DEBUG("%i distance expressions\n", exprs.size());

}

typedef OpenRAVE::RaveVector<float> RaveVectorf;

void PlotCollisions(const std::vector<Collision>& collisions, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles, double safe_dist) {
  BOOST_FOREACH(const Collision& col, collisions) {
    RaveVectorf color = (col.distance < safe_dist) ?  RaveVectorf(1,0,0,1) : RaveVectorf(0,1,0,1);
    handles.push_back(env.drawarrow(col.ptA, col.ptB, .0025, color));
  }
}

CollisionCost::CollisionCost(double dist_pen, double coeff, RobotAndDOFPtr rad, const VarVector& vars) :
    Cost("collision"),
    m_calc(rad, vars), m_dist_pen(dist_pen), m_coeff(coeff)
{}

ConvexObjectivePtr CollisionCost::convex(const vector<double>& x, Model* model) {
  ConvexObjectivePtr out(new ConvexObjective(model));
  vector<AffExpr> exprs;
  DblVec weights;
  m_calc.CalcDistExpressions(x, exprs, weights);
  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addHinge(viol, m_coeff*weights[i]);
  }
  return out;
}
double CollisionCost::value(const vector<double>& x) {
  DblVec dists, weights;
  m_calc.CalcDists(x, dists, weights);
  double out = 0;
  for (int i=0; i < dists.size(); ++i) {
    out += pospart(m_dist_pen - dists[i]) * m_coeff * weights[i];
  }
  return out;
}

void CollisionCost::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  m_calc.m_rad->SetDOFValues(getDblVec(x, m_calc.m_vars));
  vector<Collision> collisions;
  m_calc.m_cc->BodyVsAll(*m_calc.m_rad->GetRobot(), collisions);
  PlotCollisions(collisions, env, handles, m_dist_pen);
}



}
