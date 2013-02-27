#pragma once
#include <Eigen/Core>
#include <trajopt/common.hpp>
#include "sco/modeling_utils.hpp"

namespace trajopt {
using Eigen::MatrixX2d;

struct ZMPConstraint : public IneqConstraint, public Plotter {
  RobotAndDOFPtr m_rad;
  VarVector m_vars;
  MatrixX2d m_ab, m_pts;
  VectorXd m_c;
  ZMPConstraint(RobotAndDOFPtr rad, const MatrixX2d& hullpts, const VarVector& vars);
  DblVec value(const DblVec&);
  ConvexConstraintsPtr convex(const DblVec&, Model* model);
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);

};


struct StaticTorqueCost : public CostFromErrFunc {
  RobotAndDOFPtr m_rad;
  VarVector m_vars;
  StaticTorqueCost(RobotAndDOFPtr rad, const VarVector& vars, double coeff);
};

struct PECost : public CostFromErrFunc {
  RobotAndDOFPtr m_rad;
  VarVector m_vars;
  PECost(RobotAndDOFPtr rad, const VarVector& vars, double coeff);
};

struct FootHeightConstraint : public ConstraintFromFunc {
  FootHeightConstraint(RobotAndDOFPtr rad, OpenRAVE::KinBody::LinkPtr link, double height, const VarVector& vars);
};

}
