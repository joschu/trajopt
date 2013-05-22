#include "humanoids/humanoids.hpp"
#include "humanoids/hull2d.hpp"
#include "sco/expr_op_overloads.hpp"
#include "sco/expr_vec_ops.hpp"
#include "sco/modeling_utils.hpp"
#include "osgviewer/osgviewer.hpp"
#include "trajopt/common.hpp"
#include "trajopt/collision_avoidance.hpp"
#include "trajopt/kinematic_terms.hpp"
#include "trajopt/plot_callback.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/configuration_space.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/stl_to_string.hpp"
#include "utils/vector_ops.hpp"
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <openrave-core.h>
#include <openrave/openrave.h>
using namespace boost::assign;
using namespace Eigen;
using namespace OpenRAVE;
using namespace std;
using namespace trajopt;
using namespace util;

bool gEnablePlot = true;
const float DIST_PEN=.02;
const float COLL_COEFF=10;
const float VEL_COEFF = .25;
string data_dir() {
  string out = DATA_DIR;
  return out;
}


/**
import openravepy
env = openravepy.Environment()
env.Load("gfe.xml")
robot = env.GetRobots()[0]
rfoot = robot.GetLink("r_foot")
aabb = rfoot.ComputeLocalAABB(rfoot)
aabb.pos()+aabb.extents()
aabb.pos()-aabb.extents()
 */
MatrixX2d GetLocalAabbPoly() {
  MatrixX2d out(4,2);
  out << 0.17939, 0.062707,
      0.17939, -0.061646,
      -0.08246, -0.061646,
      -0.08246, 0.061646;
  return out;
}
MatrixX2d local_aabb_poly = GetLocalAabbPoly();

MatrixX2d GetLeftPoly(const RobotBase& gfe) {
  OpenRAVE::Vector v = gfe.GetLink("l_foot")->GetTransform().trans;
  v.z = 0;
  return local_aabb_poly.rowwise() + Vector2d(v.x, v.y).transpose();
}
MatrixX2d GetRightPoly(const RobotBase& gfe) {
  OpenRAVE::Vector v = gfe.GetLink("r_foot")->GetTransform().trans;
  v.z = 0;
  return local_aabb_poly.rowwise() + Vector2d(v.x, v.y).transpose();
}
template <typename MatrixT>
MatrixT concat0(const MatrixT& x, const MatrixT& y) {
  MatrixT out(x.rows() + y.rows(), x.cols());
  out.topRows(x.rows()) = x;
  out.bottomRows(y.rows()) = y;
  return out;
}
template <typename MatrixT>
void extend0(MatrixT& x, const MatrixT& y) {
  x.conservativeResize(x.rows()+y.rows(), NoChange);
  x.bottomRows(y.rows()) = y;
}

MatrixX2d GetBothPoly(const RobotBase& gfe) {
  MatrixX2d left = GetLeftPoly(gfe);
  MatrixX2d right = GetRightPoly(gfe);
  MatrixX2d verts = concat0(left, right);
  MatrixX2d hull = hull2d(verts);
  return hull;
}


void SetLeftZMP(TrajOptProb& prob, int t) {
  RobotAndDOFPtr rad = prob.GetRAD();
  prob.addConstraint(ConstraintPtr(new ZMP(rad, GetLeftPoly(*rad->GetRobot()), prob.GetVarRow(t))));
}
void SetRightZMP(TrajOptProb& prob, int t) {
  RobotAndDOFPtr rad = prob.GetRAD();
  prob.addConstraint(ConstraintPtr(new ZMP(rad, GetRightPoly(*rad->GetRobot()), prob.GetVarRow(t))));
}
void SetBothZMP(TrajOptProb& prob, int t) {
  RobotAndDOFPtr rad = prob.GetRAD();
  prob.addConstraint(ConstraintPtr(new ZMP(rad, GetBothPoly(*rad->GetRobot()), prob.GetVarRow(t))));
}
void SetLinkFixed(TrajOptProb& prob, KinBody::LinkPtr link, int t, const OpenRAVE::Transform& tlink) {
//  BoolVec allbutz(6,true);
//  allbutz[5] = false;
  prob.addConstraint(ConstraintPtr(new CartPoseConstraint(prob.GetVarRow(t), tlink, prob.GetRAD(), link)));
}
void SetLinkFixed(TrajOptProb& prob, KinBody::LinkPtr link, int t) {
  OpenRAVE::Transform tlink = link->GetTransform();
  SetLinkFixed(prob, link, t, tlink);
}
void SetStartFixed(TrajOptProb& prob) {
  DblVec cur_dofvals = prob.GetRAD()->GetDOFValues();
  VarArray& vars = prob.GetVars();
  for (int j=0; j < vars.cols(); ++j) {
    prob.addLinearConstraint(exprSub(AffExpr(vars(0,j)), cur_dofvals[j]), EQ);
  }
}


TrajArray Optimize(TrajOptProbPtr prob) {
  // optimize
  BasicTrustRegionSQP opt(prob);
  opt.improve_ratio_threshold_ = .1;
//  opt.min
  TrajArray init(prob->GetNumSteps(), prob->GetNumDOF());
  DblVec cur_dofvals = prob->GetRAD()->GetDOFValues();
  for (int i=0; i < prob->GetNumSteps(); ++i) init.row(i) = toVectorXd(cur_dofvals);
  init += DblMatrix::Random(init.rows(), init.cols()) * .01;
  opt.initialize(DblVec(init.data(), init.data()+init.rows()*init.cols()));
  if (gEnablePlot) opt.addCallback(PlotCallback(*prob));
  opt.optimize();
  return getTraj(opt.x(), prob->GetVars());
}

void LeftRightStep(RobotBasePtr gfe, TrajArray& out) {
  OpenRAVE::KinBody::LinkPtr lfoot = gfe->GetLink("l_foot"),
      rfoot = gfe->GetLink("r_foot");

  CollisionCheckerPtr cc = CollisionChecker::GetOrCreate(*gfe->GetEnv());
  vector<Collision> collisions;
  cc->AllVsAll(collisions);
//  cc->IgnoreLink(lfoot);
//  cc->IgnoreLink(rfoot);


  // determine foot placement pose
  // set up optimization problem

  int n_steps = 5;
  float step_dist = .3;

  RobotAndDOFPtr rad(new RobotAndDOF(gfe, arange(gfe->GetDOF()), OR::DOF_Transform));

  BoolVec xonly(6, false);
  xonly[3] = true;

  {
    TrajOptProbPtr prob(new TrajOptProb(n_steps, rad));
    SetStartFixed(*prob);
    prob->addCost(CostPtr(new JointVelCost(prob->GetVars(), VEL_COEFF*VectorXd::Ones(rad->GetDOF()))));


    for (int i=1; i < n_steps; ++i) {
      prob->addCost(CostPtr(new StaticTorqueCost(rad, prob->GetVarRow(i), 1)));
      prob->addCost(CostPtr(new PECost(rad, prob->GetVarRow(i), .01)));
      prob->addCost(CostPtr(new CollisionCost(DIST_PEN, COLL_COEFF, rad, prob->GetVarRow(i))));
//      prob->addConstr(ConstraintPtr(new CartPoseConstraint(prob->GetVarRow(i), Tfoottarg, rad, rfoot, xonly)));
      SetLinkFixed(*prob, lfoot, i);
      SetLeftZMP(*prob, i);
    }
    OpenRAVE::Transform Tfootnow = rfoot->GetTransform();
    OpenRAVE::Transform Tfoottarg = Tfootnow;
    Tfoottarg.trans.x = lfoot->GetTransform().trans.x + step_dist; // 30 cm forward
//    if (Tfoottarg.trans.x >= -.1 && Tfoottarg.trans.x < .1) Tfoottarg.trans.z += .05;
    SetLinkFixed(*prob, rfoot, n_steps-1, Tfoottarg);

    TrajArray traj = Optimize(prob);
    rad->SetDOFValues(toDblVec(traj.row(n_steps-1)));
    extend0(out,traj);

  }


  {
    TrajOptProbPtr prob(new TrajOptProb(n_steps, rad));
    SetStartFixed(*prob);
    prob->addCost(CostPtr(new JointVelCost(prob->GetVars(), VEL_COEFF*VectorXd::Ones(rad->GetDOF()))));
    for (int i=1; i < n_steps; ++i) {
      prob->addCost(CostPtr(new StaticTorqueCost(rad, prob->GetVarRow(i), 1)));
      prob->addCost(CostPtr(new PECost(rad, prob->GetVarRow(i), .01)));
      prob->addCost(CostPtr(new CollisionCost(DIST_PEN, COLL_COEFF, rad, prob->GetVarRow(i))));
      SetLinkFixed(*prob, lfoot, i);
      SetLinkFixed(*prob, rfoot, i);
      SetBothZMP(*prob, i);
    }
    SetRightZMP(*prob, n_steps-1);

    TrajArray traj = Optimize(prob);
    rad->SetDOFValues(toDblVec(traj.row(n_steps-1)));
    extend0(out,traj);

  }

  {
    TrajOptProbPtr prob(new TrajOptProb(n_steps, rad));
    SetStartFixed(*prob);
    prob->addCost(CostPtr(new JointVelCost(prob->GetVars(), VEL_COEFF*VectorXd::Ones(rad->GetDOF()))));
    for (int i=1; i < n_steps; ++i) {
      prob->addCost(CostPtr(new StaticTorqueCost(rad, prob->GetVarRow(i), 1)));
      prob->addCost(CostPtr(new PECost(rad, prob->GetVarRow(i), .01)));
      prob->addCost(CostPtr(new CollisionCost(DIST_PEN, COLL_COEFF, rad, prob->GetVarRow(i))));
      SetLinkFixed(*prob, rfoot, i);
      SetRightZMP(*prob, i);
    }
    OpenRAVE::Transform Tfootnow = lfoot->GetTransform();
    OpenRAVE::Transform Tfoottarg = Tfootnow;
    Tfoottarg.trans.x = rfoot->GetTransform().trans.x + step_dist; // 30 cm forward
    SetLinkFixed(*prob, lfoot, n_steps-1, Tfoottarg);

    TrajArray traj = Optimize(prob);
    rad->SetDOFValues(toDblVec(traj.row(n_steps-1)));
    extend0(out,traj);

  }


  {
    TrajOptProbPtr prob(new TrajOptProb(n_steps, rad));
    SetStartFixed(*prob);
    prob->addCost(CostPtr(new JointVelCost(prob->GetVars(), VEL_COEFF*VectorXd::Ones(rad->GetDOF()))));
    for (int i=1; i < n_steps; ++i) {
      prob->addCost(CostPtr(new StaticTorqueCost(rad, prob->GetVarRow(i), 1)));
      prob->addCost(CostPtr(new PECost(rad, prob->GetVarRow(i), .01)));
      prob->addCost(CostPtr(new CollisionCost(DIST_PEN, COLL_COEFF, rad, prob->GetVarRow(i))));
      SetLinkFixed(*prob, lfoot, i);
      SetLinkFixed(*prob, rfoot, i);
      SetBothZMP(*prob, i);
    }
    SetLeftZMP(*prob, n_steps-1);

    TrajArray traj = Optimize(prob);
    rad->SetDOFValues(toDblVec(traj.row(n_steps-1)));
    extend0(out,traj);

  }

}

void AnimateTrajectory(RobotAndDOFPtr rad, TrajArray& traj, OSGViewerPtr viewer) {
  for (int i=0; i < traj.rows(); ++i) {
    cout << "step " << i << endl;
    rad->SetDOFValues(toDblVec(traj.row(i)));
    viewer->Idle();
  }
}

int main(int argc, char* argv[]) {
  RaveInitialize(false);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  env->Load(string(getenv("HOME")) + "/Proj/darpa-proposal/drclogs.env.xml");
  env->Load(string(getenv("HOME")) + "/Proj/drc/gfe.xml");
  vector<RobotBasePtr> robots; env->GetRobots(robots);
  RobotBasePtr gfe = robots[0];
  OSGViewerPtr viewer(new OSGViewer(env));
  env->AddViewer(viewer);

  RobotAndDOFPtr rad(new RobotAndDOF(gfe, arange(gfe->GetDOF()), OR::DOF_Transform));


  // translate to starting position
  {
    OpenRAVE::Transform T;
    T.trans.y += 1;
    T.trans.z = 0.92712;
    T.trans.x -= .35;
    gfe->SetTransform(T);
  }


  TrajArray traj(0, rad->GetDOF());
  LeftRightStep(gfe, traj);
  LeftRightStep(gfe, traj);
  LeftRightStep(gfe, traj);
  LeftRightStep(gfe, traj);
  LeftRightStep(gfe, traj);

//  AnimateTrajectory(rad, traj, viewer);



  {
    gfe->SetDOFValues(DblVec(gfe->GetDOF(), 0));
    OpenRAVE::Transform T = gfe->GetTransform();
    T.rot = OR::Vector(1,0,0,.2);
    T.rot.normalize4();
    T.trans = OR::Vector(2.4, 1.4, .92712);
    gfe->SetTransform(T);
    int n_steps = 5;
    TrajOptProbPtr prob(new TrajOptProb(n_steps, rad));
    SetStartFixed(*prob);
    prob->addCost(CostPtr(new JointVelCost(prob->GetVars(), VEL_COEFF*VectorXd::Ones(rad->GetDOF()))));
    for (int i=1; i < n_steps; ++i) {
      SetLinkFixed(*prob, gfe->GetLink("l_foot"), i);
      SetLinkFixed(*prob, gfe->GetLink("r_foot"), i);
      SetLeftZMP(*prob, i);
      SetRightZMP(*prob, i);

//      prob->addCost(CostPtr(new StaticTorqueCost(rad, prob->GetVarRow(i), 1)));
//      prob->addCost(CostPtr(new PECost(rad, prob->GetVarRow(i), .01)));
//      prob->addCost(CostPtr(new CollisionCost(DIST_PEN, COLL_COEFF, rad, prob->GetVarRow(i))));
    }
    OpenRAVE::Transform buttonpose = env->GetKinBody("bigredbutton")->GetTransform();
    buttonpose.trans.z += .1;
    BoolVec justposition(6, false);
    justposition[3] = justposition[4] = justposition[5] = true;
    prob->addConstraint(ConstraintPtr(new CartPoseConstraint(prob->GetVarRow(n_steps-1), buttonpose,
        rad, gfe->GetLink("r_hand"), justposition)));
    TrajArray traj1 = Optimize(prob);
    extend0(traj, traj1);
  }

AnimateTrajectory(rad, traj, viewer);

  viewer->Idle();
ofstream trajfile("/tmp/traj.txt");
trajfile << traj << endl;

  RaveDestroy();
}
