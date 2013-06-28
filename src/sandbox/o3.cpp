#include "o3.hpp"
#include "quat_ops.hpp"
#include "sco/expr_op_overloads.hpp"
#include "sco/optimizers.hpp"
using namespace OpenRAVE;
using namespace Eigen;

namespace trajopt {

MatrixXd getW(const MatrixXd& qs, double dt) {
  MatrixXd out(qs.rows()-1, 3);
  for (int i=0; i < out.rows(); ++i) {
    out.row(i) = (2/dt)*quatLog(quatMult(quatInv(qs.row(i)), (qs.row(i+1))));
  }
  return out;
}  


AngVelCost::AngVelCost(vector<IncrementalRBPtr> rbs, const VarArray& r, double coeff) :
    m_rbs(rbs), m_r(r), m_coeff(coeff) {
  name_="angvel";
}
double AngVelCost::value(const DblVec& x) {
  MatrixXd q(m_rbs.size(),4);
  for (int i=0; i < m_rbs.size(); ++i) q.row(i) = toVector4d(m_rbs[i]->m_q);
  MatrixXd rvals = getTraj(x, m_r);
  MatrixXd qnew(q.rows(), q.cols());
  for (int i = 0; i < qnew.rows(); ++i) {
    qnew.row(i) = quatMult(quatExp(rvals.row(i)), q.row(i));
  }
  MatrixXd wvals = getW(qnew, 1);
  return wvals.array().square().sum()*m_coeff;
}

ConvexObjectivePtr AngVelCost::convex(const DblVec& x, Model* model) {
  MatrixXd q(m_rbs.size(),4);
  for (int i=0; i < m_rbs.size(); ++i) q.row(i) = toVector4d(m_rbs[i]->m_q);
  ConvexObjectivePtr out(new ConvexObjective(model));
  MatrixXd wvals = getW(q, 1);
  for (int i = 0; i < wvals.rows(); ++i) {
    for (int j = 0; j < wvals.cols(); ++j) {
      out->addQuadExpr(exprMult(exprSquare(m_r(i + 1, j) - m_r(i, j) + wvals(i, j)), m_coeff));
    }
  }
  return out;
}



O3Helper::O3Helper(const KinBodyPtr body, const VarArray& vars) : m_body(body), m_orivars(vars) {
  int T = vars.rows();
  m_rbs.reserve(T);
  for (int i=0; i < T; ++i) {
    m_rbs.push_back(IncrementalRBPtr(new IncrementalRB(body)));
  } 
}

void O3Helper::OptimizerCallback(OptProb*, DblVec& x) {
  Eigen::MatrixXd orivals = getTraj(x, m_orivars);
  for (int i=0; i < m_rbs.size(); ++i) {
    m_rbs[i]->m_q = OR::geometry::quatMultiply(geometry::quatFromAxisAngle(OR::Vector(orivals(i,0), orivals(i,1), orivals(i,2))),m_rbs[i]->m_q);
    m_rbs[i]->m_r = OR::Vector(0,0,0);
  }
  setVec(x, m_orivars.m_data, DblVec(m_orivars.size(), 0));
}

void O3Helper::AddAngVelCosts(OptProb& prob, double coeff) {
  prob.addCost(CostPtr(new AngVelCost(m_rbs, m_orivars, coeff)));  
}
void O3Helper::ConfigureOptimizer(Optimizer& opt) {
  opt.addCallback(boost::bind(&O3Helper::OptimizerCallback, this, _1, _2));  
}


}
