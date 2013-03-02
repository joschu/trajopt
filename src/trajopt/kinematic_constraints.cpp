#include "utils/stl_to_string.hpp"
#include "trajopt/kinematic_constraints.hpp"
#include "trajopt/utils.hpp"
#include "trajopt/rave_utils.hpp"
#include "utils/logging.hpp"
#include "sco/expr_ops.hpp"
#include "sco/modeling_utils.hpp"
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <Eigen/Geometry>
#include <openrave/openrave.h>
#include "utils/eigen_conversions.hpp"
#include "utils/eigen_slicing.hpp"
using namespace std;
using namespace sco;
using namespace Eigen;
using namespace util;

namespace {


static MatrixXd diffAxis0(const MatrixXd& in) {
  return in.middleRows(1, in.rows()-1) - in.middleRows(0, in.rows()-1);
}
Vector3d rotVec(const Matrix3d& m) {
  Quaterniond q; q = m;
  return Vector3d(q.x(), q.y(), q.z());
}
Vector3d rotVec(const OpenRAVE::Vector& q) {
  return Vector3d(q[1], q[2], q[3]);
}

VectorXd concat(const VectorXd& a, const VectorXd& b) {
  VectorXd out(a.size()+b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}


}

namespace trajopt {

//////////////////////



void makeTrajVariablesAndBounds(int n_steps, RobotAndDOFPtr manip, OptProb& prob_out, VarArray& vars_out) {
  int n_dof = manip->GetDOF();
  DblVec lower, upper;
  manip->GetDOFLimits(lower, upper);
  LOG_INFO("Dof limits: %s, %s", CSTR(lower), CSTR(upper));
  vector<double> vlower, vupper;
  vector<string> names;
  for (int i=0; i < n_steps; ++i) {
    vlower.insert(vlower.end(), lower.data(), lower.data()+lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data()+upper.size());
    for (unsigned j=0; j < n_dof; ++j) {
      names.push_back( (boost::format("j_%i_%i")%i%j).str() );
    }
  }

  prob_out.createVariables(names, vlower, vupper);
  vars_out = VarArray(n_steps, n_dof, prob_out.getVars().data());

}

JointPosCost::JointPosCost(const VarVector& vars, const VectorXd& vals, const VectorXd& coeffs) :
    Cost("JointVel"), vars_(vars), vals_(vals), coeffs_(coeffs) {
    for (int i=0; i < vars.size(); ++i) {
      if (coeffs[i] > 0) {
        AffExpr diff = exprSub(AffExpr(vars[i]), AffExpr(vals[i]));
        exprInc(expr_, exprMult(exprSquare(diff), coeffs[i]));
      }
    }
}
double JointPosCost::value(const vector<double>& xvec) {
  VectorXd dofs = getVec(xvec, vars_);
  return ((dofs - vals_).array().square() * coeffs_.array()).sum();
}
ConvexObjectivePtr JointPosCost::convex(const vector<double>& x, Model* model) {
  ConvexObjectivePtr out(new ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}


JointVelCost::JointVelCost(const VarArray& vars, const VectorXd& coeffs) :
    Cost("JointVel"), vars_(vars), coeffs_(coeffs) {
  for (int i=0; i < vars.rows()-1; ++i) {
    for (int j=0; j < vars.cols(); ++j) {
      AffExpr vel;
      exprInc(vel, exprMult(vars(i,j), -1));
      exprInc(vel, exprMult(vars(i+1,j), 1));
      exprInc(expr_, exprMult(exprSquare(vel),coeffs_[j]));
    }
  }
}
double JointVelCost::value(const vector<double>& xvec) {
  MatrixXd traj = getTraj(xvec, vars_);
  return (diffAxis0(traj).array().square().matrix() * coeffs_.asDiagonal()).sum();
}
ConvexObjectivePtr JointVelCost::convex(const vector<double>& x, Model* model) {
  ConvexObjectivePtr out(new ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}



JointAccCost::JointAccCost(const VarArray& vars, const VectorXd& coeffs) :
    Cost("JointAcc"), vars_(vars), coeffs_(coeffs) {
  for (int i=0; i < vars.rows()-2; ++i) {
    for (int j=0; j < vars.cols(); ++j) {
      AffExpr acc;
      exprInc(acc, exprMult(vars(i,j), -1));
      exprInc(acc, exprMult(vars(i+1,j), 2));
      exprInc(acc, exprMult(vars(i+2,j), -1));
      exprInc(expr_, exprMult(exprSquare(acc), coeffs_[j]));
    }
  }
}
double JointAccCost::value(const vector<double>& xvec) {
  MatrixXd traj = getTraj(xvec, vars_);
  return (diffAxis0(diffAxis0(traj)).array().square().matrix() * coeffs_.asDiagonal()).sum();
}
ConvexObjectivePtr JointAccCost::convex(const vector<double>& x, Model* model) {
  ConvexObjectivePtr out(new ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}




struct CartPoseErrCalculator : public VectorOfVector {
  OR::Transform pose_inv_;
  RobotAndDOFPtr manip_;
  OR::KinBody::LinkPtr link_;
  VectorXd coeffs_;
  CartPoseErrCalculator(const OR::Transform& pose, RobotAndDOFPtr manip, OR::KinBody::LinkPtr link, const VectorXd& coeffs) :
  pose_inv_(pose.inverse()),
  manip_(manip),
  link_(link),
  coeffs_(coeffs)
  {}
//  CartPoseCostCalculator(const CartPoseCostCalculator& other) : pose_(other.pose_), manip_(other.manip_), rs_(other.rs_) {}
  VectorXd operator()(const VectorXd& dof_vals) const {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();

    OR::Transform pose_err = pose_inv_ * newpose;
    VectorXd err = coeffs_.cwiseProduct(concat(rotVec(pose_err.rot), toVector3d(pose_err.trans)));
    return err;
  }
};

CartPoseCost::CartPoseCost(const VarVector& vars, const OR::Transform& pose, const Vector3d& rot_coeffs,
    const Vector3d& pos_coeffs, RobotAndDOFPtr manip, KinBody::LinkPtr link) :
    CostFromErrFunc(VectorOfVectorPtr(new CartPoseErrCalculator(pose, manip, link, VectorXd::Ones(6))),
        vars, concat(rot_coeffs, pos_coeffs), ABS, "CartPose")
{}
void CartPoseCost::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  CartPoseErrCalculator* calc = static_cast<CartPoseErrCalculator*>(f_.get());
  DblVec dof_vals = getDblVec(x, vars_);
  calc->manip_->SetDOFValues(dof_vals);
  OR::Transform target = calc->pose_inv_.inverse(), cur = calc->link_->GetTransform();
  PlotAxes(env, cur, .1,  handles);
  PlotAxes(env, target, .1,  handles);
  handles.push_back(env.drawarrow(cur.trans, target.trans, .01, OR::Vector(1,0,1,1)));
}


CartPoseConstraint::CartPoseConstraint(const VarVector& vars, const OR::Transform& pose,
    RobotAndDOFPtr manip, KinBody::LinkPtr link, const VectorXd& coeffs) :
    ConstraintFromFunc(VectorOfVectorPtr(new CartPoseErrCalculator(pose, manip, link, coeffs)),
        vars, EQ, "CartPose")
{
}

void CartPoseConstraint::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  // IDENTITCAL TO CartPoseCost::Plot
  CartPoseErrCalculator* calc = static_cast<CartPoseErrCalculator*>(f_.get());
  DblVec dof_vals = getDblVec(x, vars_);
  calc->manip_->SetDOFValues(dof_vals);
  OR::Transform target = calc->pose_inv_.inverse(), cur = calc->link_->GetTransform();
  PlotAxes(env, cur, .1,  handles);
  PlotAxes(env, target, .1,  handles);
  handles.push_back(env.drawarrow(cur.trans, target.trans, .01, OR::Vector(1,0,1,1)));
}


struct CartPositionErrCalculator {
  Vector3d pt_world_;
  RobotAndDOFPtr manip_;
  OR::KinBody::LinkPtr link_;
  CartPositionErrCalculator(const Vector3d& pt_world, RobotAndDOFPtr manip, OR::KinBody::LinkPtr link) :
  pt_world_(pt_world),
  manip_(manip),
  link_(link)
  {}
  VectorXd operator()(const VectorXd& dof_vals) {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();
    return pt_world_ - toVector3d(newpose.trans);
  }
};

struct CartVelJacCalculator : MatrixOfVector {
  RobotAndDOFPtr manip_;
  KinBody::LinkPtr link_;
  double limit_;
  CartVelJacCalculator(RobotAndDOFPtr manip, KinBody::LinkPtr link, double limit) :
    manip_(manip), link_(link), limit_(limit) {}

  MatrixXd operator()(const VectorXd& dof_vals) const {
    int n_dof = manip_->GetDOF();
    MatrixXd out(6, 2*n_dof);
    manip_->SetDOFValues(toDblVec(dof_vals.topRows(n_dof)));
    OR::Transform pose0 = link_->GetTransform();
    MatrixXd jac0 = manip_->PositionJacobian(link_->GetIndex(), pose0.trans);
    manip_->SetDOFValues(toDblVec(dof_vals.bottomRows(n_dof)));
    OR::Transform pose1 = link_->GetTransform();
    MatrixXd jac1 = manip_->PositionJacobian(link_->GetIndex(), pose1.trans);
    out.block(0,0,3,n_dof) = -jac0;
    out.block(0,n_dof,3,n_dof) = jac1;
    out.block(3,0,3,n_dof) = jac0;
    out.block(3,n_dof,3,n_dof) = -jac1;
    return out;
  }
};

struct CartVelCalculator : VectorOfVector {
  RobotAndDOFPtr manip_;
  KinBody::LinkPtr link_;
  double limit_;
  CartVelCalculator(RobotAndDOFPtr manip, KinBody::LinkPtr link, double limit) :
    manip_(manip), link_(link), limit_(limit) {}

  VectorXd operator()(const VectorXd& dof_vals) const {
    int n_dof = manip_->GetDOF();
    manip_->SetDOFValues(toDblVec(dof_vals.topRows(n_dof)));
    OR::Transform pose0 = link_->GetTransform();
    manip_->SetDOFValues(toDblVec(dof_vals.bottomRows(n_dof)));
    OR::Transform pose1 = link_->GetTransform();
    VectorXd out(6);
    out.topRows(3) = toVector3d(pose1.trans - pose0.trans - OR::Vector(limit_,limit_,limit_));
    out.bottomRows(3) = toVector3d( - pose1.trans + pose0.trans - OR::Vector(limit_, limit_, limit_));
    return out;
  }
};

CartVelConstraint::CartVelConstraint(const VarVector& step0vars, const VarVector& step1vars, RobotAndDOFPtr manip, KinBody::LinkPtr link, double distlimit) :
        ConstraintFromFunc(VectorOfVectorPtr(new CartVelCalculator(manip, link, distlimit)),
                           MatrixOfVectorPtr(new CartVelJacCalculator(manip, link, distlimit)),
                          concat(step0vars, step1vars), INEQ, "CartVel")
{}



struct UpErrorCalculator {
  Vector3d dir_local_;
  Vector3d goal_dir_world_;
  RobotAndDOFPtr manip_;
  OR::KinBody::LinkPtr link_;
  MatrixXd perp_basis_; // 2x3 matrix perpendicular to goal_dir_world
  UpErrorCalculator(const Vector3d& dir_local, const Vector3d& goal_dir_world, RobotAndDOFPtr manip, KinBody::LinkPtr link) :
    dir_local_(dir_local),
    goal_dir_world_(goal_dir_world),
    manip_(manip),
    link_(link)
  {
    Vector3d perp0 = goal_dir_world_.cross(Vector3d::Random()).normalized();
    Vector3d perp1 = goal_dir_world_.cross(perp0);
    perp_basis_.resize(2,3);
    perp_basis_.row(0) = perp0.transpose();
    perp_basis_.row(1) = perp1.transpose();
  }
  VectorXd operator()(const VectorXd& dof_vals) {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();
    return perp_basis_*(toRot(newpose.rot) * dir_local_ - goal_dir_world_);
  }
};
}
