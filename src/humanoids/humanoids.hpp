#pragma once
#include <Eigen/Core>
#include <trajopt/common.hpp>
#include "ipi/sco/modeling_utils.hpp"

namespace trajopt {
using Eigen::MatrixX2d;

struct ZMP: public Constraint, public Plotter {
  RobotAndDOFPtr m_rad;
  VarVector m_vars;
  MatrixX2d m_ab, m_pts;
  VectorXd m_c;
  ZMP(RobotAndDOFPtr rad, const MatrixX2d& hullpts, const VarVector& vars);
  DblVec value(const DblVec&);
  ConvexConstraintsPtr convex(const DblVec&, Model* model);
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
  ConstraintType type() {return INEQ;}

};


struct StaticTorque : public CostFromNumDiffErr {
  RobotAndDOFPtr m_rad;
  VarVector m_vars;
  StaticTorque(RobotAndDOFPtr rad, const VarVector& vars, double coeff);
};

struct PECost : public CostFromNumDiffErr {
  RobotAndDOFPtr m_rad;
  VarVector m_vars;
  PECost(RobotAndDOFPtr rad, const VarVector& vars, double coeff);
};

}
