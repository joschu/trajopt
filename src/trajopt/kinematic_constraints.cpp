#include "utils/stl_to_string.hpp"
#include "trajopt/kinematic_constraints.hpp"
#include "trajopt/utils.hpp"
#include "trajopt/rave_utils.hpp"
#include "ipi/logging.hpp"
#include "ipi/sco/expr_ops.hpp"
#include "ipi/sco/modeling_utils.hpp"
#include <boost/format.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <Eigen/Geometry>
#include <openrave/openrave.h>
#include "utils/eigen_conversions.hpp"
#include "utils/eigen_slicing.hpp"
using namespace std;
using namespace ipi::sco;
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
  CartPoseErrCalculator(const OR::Transform& pose, RobotAndDOFPtr manip, OR::KinBody::LinkPtr link) :
  pose_inv_(pose.inverse()),
  manip_(manip),
  link_(link)
  {}
//  CartPoseCostCalculator(const CartPoseCostCalculator& other) : pose_(other.pose_), manip_(other.manip_), rs_(other.rs_) {}
  VectorXd operator()(const VectorXd& dof_vals) const {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();

    OR::Transform pose_err = pose_inv_ * newpose;
    VectorXd err = concat(rotVec(pose_err.rot), toVector3d(pose_err.trans));
    return err;
  }
};

CartPoseCost::CartPoseCost(const VarVector& vars, const OR::Transform& pose, const Vector3d& rot_coeffs,
    const Vector3d& pos_coeffs, RobotAndDOFPtr manip, KinBody::LinkPtr link) :
    CostFromNumDiffErr(VectorOfVectorPtr(new CartPoseErrCalculator(pose, manip, link)),
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
    RobotAndDOFPtr manip, KinBody::LinkPtr link, const BoolVec& enabled) :
    ConstraintFromNumDiff(VectorOfVectorPtr(new CartPoseErrCalculator(pose, manip, link)),
        vars, EQ, "CartPose", enabled)
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
//  CartPoseCostCalculator(const CartPoseCostCalculator& other) : pose_(other.pose_), manip_(other.manip_), rs_(other.rs_) {}
  VectorXd operator()(const VectorXd& dof_vals) {
    manip_->SetDOFValues(toDblVec(dof_vals));
    OR::Transform newpose = link_->GetTransform();
    return pt_world_ - toVector3d(newpose.trans);
  }
};

#if 0
class CartPositionConstraint : public CostFromNumDiffErr, public Plottable {
  CartPositionConstraint(const VarVector& vars, const Vector3d& pt, RobotAndDOFPtr manip, KinBody::LinkPtr link) :
    CostFromNumDiffErr(CartPositionErrCalculator(pt, manip, link), vars, EQ, "CartPosition") {}
};
#endif


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
    out.bottomRows(3) = toVector3d(OR::Vector(limit_,limit_,limit_) - pose1.trans + pose0.trans);
    return out;
  }
};

CartVelConstraint::CartVelConstraint(const VarVector& step0vars, const VarVector& step1vars, RobotAndDOFPtr manip, KinBody::LinkPtr link, double distlimit) :
        ConstraintFromNumDiff(VectorOfVectorPtr(new CartVelCalculator(manip, link, distlimit)),
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

#if 0
CostPtr makeUpCost(const VarVector& vars, const Vector3d& dir_local, const Vector3d& goal_dir_world, double coeff, RobotAndDOFPtr manip, KinBody::LinkPtr link) {
  VectorOfVector f = UpErrorCalculator(dir_local, goal_dir_world, manip, link);
  // why does this work without a copy constructor?
  return CostPtr(new CostFromNumDiffErr(f, vars, coeff*Vector2d::Ones(), ABS, "Up"));
}

ConstraintPtr makeUpConstraint(const VarVector& vars, const Vector3d& dir_local, const Vector3d& goal_dir_world, RobotAndDOFPtr manip, KinBody::LinkPtr link) {
  VectorOfVector f = UpErrorCalculator(dir_local, goal_dir_world, manip, link);
  return ConstraintPtr(new ConstraintFromNumDiff(f, vars, EQ, "Up"));
}
#endif

