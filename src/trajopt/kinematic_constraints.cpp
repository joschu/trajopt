#include "utils/general_utils.hpp"
#include "trajopt/kinematic_constraints.hpp"
#include "trajopt/utils.hpp"
#include "ipi/logging.hpp"
#include "ipi/sco/expr_ops.hpp"
#include "ipi/sco/modeling_utils.hpp"
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <Eigen/Geometry>
#include <openrave/openrave.h>
using namespace std;
using namespace ipi::sco;
using namespace Eigen;
using namespace util;


namespace trajopt {

//////////////////////

Vector3d toVector3d(const OR::Vector& v) {
  return Vector3d(v.x, v.y, v.z);
}
Eigen::Matrix3d toRot(const OR::Vector& rq) {
  Eigen::Affine3d T;
  T = Eigen::Quaterniond(rq[0], rq[1], rq[2], rq[3]);
  return T.rotation();
}


void makeTrajVariablesAndBounds(int n_steps, RobotAndDOFPtr manip, OptProb& prob_out, VarArray& vars_out) {
  int n_dof = manip->GetDOF();
  DblVec lower, upper;
  manip->GetDOFLimits(lower, upper);
  IPI_LOG_INFO("Dof limits: %s, %s", Str(lower), Str(upper));
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


JointVelCost::JointVelCost(const VarArray& vars, const VectorXd& coeffs) : vars_(vars), coeffs_(coeffs) {
  for (int i=0; i < vars.rows()-1; ++i) {
    for (int j=0; j < vars.cols(); ++j) {
      AffExpr vel;
      exprInc(vel, exprMult(vars(i,j), -1));
      exprInc(vel, exprMult(vars(i+1,j), 1));
      exprInc(expr_, exprMult(exprSquare(vel),coeffs_[j]));
    }
  }
}

static MatrixXd diffAxis0(const MatrixXd& in) {
  return in.middleRows(1, in.rows()-1) - in.middleRows(0, in.rows()-1);
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



JointAccCost::JointAccCost(const VarArray& vars, const VectorXd& coeffs) : vars_(vars), coeffs_(coeffs) {
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
//
//#if 0
//VectorXd flattenPose(const Matrix3d& r, const Vector3d& t) {
//  VectorXd out(12);
//  out.topRows(9) = flattenRowwise(r);
//  out.bottomRows(3) = t;
//  return out;
//}
//#else
//#if 0
//VectorXd flattenPose(const Matrix3d& r, const Vector3d& t) {
//  Quaterniond q; q = r;
//  VectorXd out(6);
//  out.topRows(3) = Vector3d(q.x(), q.y(), q.z());
//  out.bottomRows(3) = t;
//  return out;
//}
//
//#endif

Vector3d rotVec(const Matrix3d& m) {
  Quaterniond q; q = m;
  return Vector3d(q.x(), q.y(), q.z());
}
Vector3d rotVec(const OR::Vector& q) {
  return Vector3d(q[1], q[2], q[3]);
}


struct CartPoseErrCalculator {
  OR::Transform pose_inv_;
  RobotAndDOFPtr manip_;
  OR::KinBody::LinkPtr link_;
  CartPoseErrCalculator(const OR::Transform& pose, RobotAndDOFPtr manip, OR::KinBody::LinkPtr link) :
  pose_inv_(pose.inverse()),
  manip_(manip),
  link_(link)
  {}
//  CartPoseCostCalculator(const CartPoseCostCalculator& other) : pose_(other.pose_), manip_(other.manip_), rs_(other.rs_) {}
  VectorXd operator()(const VectorXd& dof_vals) {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();

    OR::Transform pose_err = newpose * pose_inv_;
    VectorXd err(6);
    err << rotVec(pose_err.rot), toVector3d(pose_err.trans);
    return err;
  }
};


CostPtr makeCartPoseCost(const VarVector& vars, const OR::Transform& pose, const Vector3d& rot_coeffs, const Vector3d& pos_coeffs, RobotAndDOFPtr manip, KinBody::LinkPtr link) {
  VectorOfVector f = CartPoseErrCalculator(pose, manip, link);
  VectorXd coeffs(6);
  coeffs << rot_coeffs, pos_coeffs;
  return CostPtr(new CostFromNumDiffErr(f, vars, coeffs, ABS, "CartPose"));
}

ConstraintPtr makeCartPoseConstraint(const VarVector& vars, const OR::Transform& pose, RobotAndDOFPtr manip, KinBody::LinkPtr link) {
  VectorOfVector f = CartPoseErrCalculator(pose, manip, link);
  return ConstraintPtr(new ConstraintFromNumDiff(f, vars, EQ, "CartPose"));
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
//  CartPoseCostCalculator(const CartPoseCostCalculator& other) : pose_(other.pose_), manip_(other.manip_), rs_(other.rs_) {}
  VectorXd operator()(const VectorXd& dof_vals) {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();
    return pt_world_ - toVector3d(newpose.trans);
  }
};

ConstraintPtr makeCartPositionConstraint(const VarVector& vars, const Vector3d& pt, RobotAndDOFPtr manip, KinBody::LinkPtr link) {
  VectorOfVector f = CartPositionErrCalculator(pt, manip, link);
  return ConstraintPtr(new ConstraintFromNumDiff(f, vars, EQ, "CartPosition"));
}


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

CostPtr makeUpCost(const VarVector& vars, const Vector3d& dir_local, const Vector3d& goal_dir_world, double coeff, RobotAndDOFPtr manip, KinBody::LinkPtr link) {
  VectorOfVector f = UpErrorCalculator(dir_local, goal_dir_world, manip, link);
  // why does this work without a copy constructor?
  return CostPtr(new CostFromNumDiffErr(f, vars, coeff*Vector2d::Ones(), ABS, "Up"));
}

ConstraintPtr makeUpConstraint(const VarVector& vars, const Vector3d& dir_local, const Vector3d& goal_dir_world, RobotAndDOFPtr manip, KinBody::LinkPtr link) {
  VectorOfVector f = UpErrorCalculator(dir_local, goal_dir_world, manip, link);
  return ConstraintPtr(new ConstraintFromNumDiff(f, vars, EQ, "Up"));
}


}
