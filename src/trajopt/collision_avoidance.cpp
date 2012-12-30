#include "trajopt/collision_avoidance.hpp"
#include <boost/foreach.hpp>
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "ipi/sco/expr_vec_ops.hpp"
#include "ipi/sco/expr_ops.hpp"
using namespace OpenRAVE;
using namespace ipi::sco;
using namespace util;

namespace trajopt {

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(RobotAndDOFPtr rad, const VarVector& vars) :
  m_rad(rad), m_vars(vars) {

  RobotBasePtr robot = rad->GetRobot();
  const vector<KinBody::LinkPtr>& links = robot->GetLinks();
  const std::set<int>& pairhashes = robot->GetAdjacentLinks();
  BOOST_FOREACH(const int& p, pairhashes) {
    m_ignorer.ExcludePair(*robot_links[p & 0xffff], *robot_links[p >> 16]);
  }
  vector<KinBody::LinkPtr> links;
  vector<int> inds;
  GetAffectedLinks(links, inds);
  for (int i=0; i < links.size(); ++i) {
    m_link2ind[links[i].get()] = inds[i];
  }

}

void SingleTimestepCollisionEvaluator::CalcDistExpressions(vector<AffExpr>& exprs, DblVec& weights) {
  vector<Collision> collisions;
  m_cc->AllVsAll(&m_ignorer, collisions);
  exprs.reserve(expr.size() + collisions.size());
  weights.reserve(weights.size() + collisions.size());
  vector<double> dofvals = m_rad->GetDOFValues();

  BOOST_FOREACH(const Collision& col, collisions) {
    AffExpr dist;
    map<string, int>::iterator itA = m_link2ind.find(col.linkA);
    if (itA != m_link2ind.end()) {
      VectorXd dist_grad = toVector3d(col.normal).transpose()*m_rad->PositionJacobian(itA->second, col.ptA);
      exprInc(dist, varDot(depth_grad, m_vars));
      exprInc(dist, -depth_grad.dot(dofvals));
    }
    map<string, int>::iterator itB = m_link2ind.find(col.linkB);
    if (itB != m_link2ind.end()) {
      VectorXd dist_grad = -toVector3d(col.normal).transpose()*m_rad->PositionJacobian(itB->second, col.ptB);
      exprInc(dist, varDot(depth_grad, m_vars));
      exprInc(dist, -depth_grad.dot(dofvals));
    }
  }

}


}
