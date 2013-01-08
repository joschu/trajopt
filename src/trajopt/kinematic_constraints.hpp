#pragma once
#include "ipi/sco/modeling.hpp"
#include "ipi/sco/modeling_utils.hpp"
#include "ipi/sco/sco_fwd.hpp"
#include <Eigen/Core>
#include "trajopt/common.hpp"
#include <openrave/openrave.h>
namespace trajopt {

using namespace ipi::sco;
typedef BasicArray<Var> VarArray;

void makeTrajVariablesAndBounds(int n_steps, const RobotAndDOF& manip, OptProb& prob_out, VarArray& vars_out);

#if 0
CostPtr makeUpCost(const VarVector& vars, const Vector3d& dir_local, const Vector3d& goal_dir_world, double coeff, RobotAndDOFPtr manip, KinBody::LinkPtr link);
ConstraintPtr makeUpConstraint(const VarVector& vars, const Vector3d& dir_local, const Vector3d& goal_dir_world, RobotAndDOFPtr manip, KinBody::LinkPtr link);
#endif

class CartPoseCost : public CostFromNumDiffErr, public Plotter {
public:
  CartPoseCost(const VarVector& vars, const OR::Transform& pose, const Vector3d& rot_coeffs, const Vector3d& pos_coeffs, RobotAndDOFPtr manip, KinBody::LinkPtr link);
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};
class CartPoseConstraint : public ConstraintFromNumDiff, public Plotter {
public:
  CartPoseConstraint(const VarVector& vars, const OR::Transform& pose, RobotAndDOFPtr manip, KinBody::LinkPtr link, const BoolVec& enabled=BoolVec());
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};

#if 0
class CartPositionCost : public CostFromNumDiffErr, public Plottable {
  CartPoseCost(const VarVector& vars, const OR::Vector& position, const Vector3d& pos_coeffs, RobotAndDOFPtr manip, KinBody::LinkPtr link);
};
class CartPositionConstraint : public ConstraintFromNumDiffErr, public Plottable {
  CartPositionConstraint(const VarVector& vars, const Vector3d& pt, RobotAndDOFPtr manip, KinBody::LinkPtr link);
};
#endif



class JointVelCost : public Cost {
public:
  JointVelCost(const VarArray& traj, const VectorXd& coeffs);
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  virtual double value(const vector<double>&);
private:
  VarArray vars_;
  VectorXd coeffs_;
  QuadExpr expr_;
};


class JointAccCost : public Cost {
public:
  JointAccCost(const VarArray& traj, const VectorXd& coeffs);
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  virtual double value(const vector<double>&);
private:
  VarArray vars_;
  VectorXd coeffs_;
  QuadExpr expr_;
};


}
