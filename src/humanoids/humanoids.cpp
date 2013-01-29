#include "humanoids.hpp"
#include "ipi/sco/modeling_utils.hpp"
#include "ipi/sco/expr_ops.hpp"
#include "ipi/sco/expr_vec_ops.hpp"
#include "ipi/sco/expr_op_overloads.hpp"
#include <boost/foreach.hpp>
#include "utils/eigen_conversions.hpp"
using namespace util;
using namespace Eigen;
using namespace std;
namespace trajopt {

void PolygonToEquations(const MatrixX2d& pts, MatrixX2d& ab, VectorXd& c) {
  // ax + by + c <= 0
  // assume polygon is convex
  ab.resize(pts.rows(),2);
  c.resize(pts.rows());
  Vector2d p0 = pts.row(0);

  for (int i=0; i < pts.rows(); ++i) {
    int i1 = (i+1) % pts.rows();
    double x0 = pts(i,0),
        y0 = pts(i,1),
        x1 = pts(i1,0),
        y1 = pts(i1,1);
    ab(i,0) = -(y1 - y0);
    ab(i,1) = x1 - x0;
    ab.row(i).normalize();
    c(i) = -ab.row(i).dot(pts.row(i));
  }

  Vector2d centroid = pts.colwise().mean();
  if (ab.row(0) * centroid + c(0) >= 0) {
    ab *= -1;
    c *= -1;
  }

}


ZMP::ZMP(RobotAndDOFPtr rad, const MatrixX2d& hullpts, const VarVector& vars) :
          m_rad(rad), m_vars(vars), m_pts(hullpts) {

  // find the equations representing the polygon
  PolygonToEquations(m_pts, m_ab, m_c);
}

DblVec ZMP::value(const DblVec& x) {
  m_rad->SetDOFValues(getDblVec(x,m_vars));
  OR::Vector moment(0,0,0);
  // calculate center of mass
  float totalmass = 0;
  BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
    if (!link->GetGeometries().empty()) {
      moment += link->GetGlobalMassFrame().trans * link->GetMass();
      totalmass += link->GetMass();
    }
  }
  moment /= totalmass;
  Vector2d xy;
  xy(0) = moment.x;
  xy(1) = moment.y;
  return toDblVec(m_ab * xy + m_c);
}

ConvexConstraintsPtr ZMP::convex(const DblVec& x, Model* model) {
  DblVec curvals = getDblVec(x, m_vars);
  m_rad->SetDOFValues(curvals);
  DblMatrix jacmoment = DblMatrix::Zero(3, curvals.size());
  OR::Vector moment(0,0,0);
  float totalmass = 0;
  BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
    if (!link->GetGeometries().empty()) {
      OR::Vector cm = link->GetGlobalMassFrame().trans;
      moment += cm * link->GetMass();
      jacmoment += m_rad->PositionJacobian(link->GetIndex(), cm) * link->GetMass();
      totalmass += link->GetMass();
    }
  }
  moment /= totalmass;
  jacmoment /= totalmass;

  AffExpr x_expr = AffExpr(moment.x) + varDot(jacmoment.row(0), m_vars) - jacmoment.row(0).dot(toVectorXd(curvals));
  AffExpr y_expr = AffExpr(moment.y) + varDot(jacmoment.row(1), m_vars) - jacmoment.row(1).dot(toVectorXd(curvals));

  ConvexConstraintsPtr out(new ConvexConstraints(model));
  for (int i=0; i < m_ab.rows(); ++i) {
    out->addIneqCnt(m_ab(i,0) * x_expr + m_ab(i,1) * y_expr + m_c(i));
  }
  return out;
}

void ZMP::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  Matrix<float, Dynamic, 3, RowMajor> xyz(m_ab.rows()*2, 3);
  xyz.col(2).setZero();
  int npts = m_pts.rows();
  for (int i=0; i < npts; ++i) {
    xyz(2*i,0) = m_pts(i,0);
    xyz(2*i,1) = m_pts(i,1);
    xyz(2*i+1,0) = m_pts((i+1)%npts,0);
    xyz(2*i+1,1) = m_pts((i+1)%npts,1);
  }
  handles.push_back(env.drawlinelist(xyz.data(), xyz.rows(), sizeof(float)*3, 4, OR::RaveVector<float>(1,0,0,1)));

  m_rad->SetDOFValues(getDblVec(x,m_vars));
  OR::Vector moment(0,0,0);
  float totalmass = 0;
  BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
    if (!link->GetGeometries().empty()) {
      moment += link->GetGlobalMassFrame().trans * link->GetMass();
      totalmass += link->GetMass();
    }
  }
  moment /= totalmass;
  OR::Vector moment_ground = moment; moment_ground.z = 0;
  handles.push_back(env.drawarrow(moment, moment_ground, .005, OR::RaveVector<float>(1,0,0,1)));
}

struct StaticTorqueCalc : public VectorOfVector {
  RobotAndDOFPtr m_rad;
  StaticTorqueCalc(const RobotAndDOFPtr rad) : m_rad(rad) {}
  VectorXd operator()(const VectorXd& x) const {
    m_rad->SetDOFValues(toDblVec(x));
    VectorXd out = VectorXd::Zero(m_rad->GetDOF());
    BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
      if (!link->GetGeometries().empty()) {
        OR::Vector cm = link->GetGlobalMassFrame().trans;
        DblMatrix jac = m_rad->PositionJacobian(link->GetIndex(), cm) * link->GetMass();
        out += jac.row(2).transpose();
      }
    }
    return out;
  }
};

StaticTorque::StaticTorque(RobotAndDOFPtr rad, const VarVector& vars, double coeff) :
  CostFromNumDiffErr(VectorOfVectorPtr(new StaticTorqueCalc(rad)), vars, VectorXd::Ones(vars.size())*coeff,
    SQUARED,  "static_torque") {
}

struct PECalc : public VectorOfVector {
  RobotAndDOFPtr m_rad;
  PECalc(const RobotAndDOFPtr rad) : m_rad(rad) {}
  VectorXd operator()(const VectorXd& x) const {
    m_rad->SetDOFValues(toDblVec(x));
    VectorXd out = VectorXd::Zero(1);
    BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
      if (!link->GetGeometries().empty()) {
        OR::Vector cm = link->GetGlobalMassFrame().trans;
        out(0) += cm.z;
      }
    }
    return out;
  }
};

PECost::PECost(RobotAndDOFPtr rad, const VarVector& vars, double coeff) :
  CostFromNumDiffErr(VectorOfVectorPtr(new PECalc(rad)), vars, VectorXd::Ones(1)*coeff,
    SQUARED,  "PE") {
}


}
