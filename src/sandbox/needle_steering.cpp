#include "o3.hpp"
#include "sco/expr_ops.hpp"
#include "sco/modeling_utils.hpp"
#include "osgviewer/osgviewer.hpp"
#include "trajopt/collision_checker.hpp"
#include "trajopt/collision_terms.hpp"
#include "trajopt/common.hpp"
#include "trajopt/plot_callback.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/trajectory_costs.hpp"
#include "utils/clock.hpp"
#include "utils/config.hpp"
#include "utils/eigen_conversions.hpp"
#include "utils/stl_to_string.hpp"
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <ctime>
#include <openrave-core.h>
#include <openrave/openrave.h>

using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;
using namespace Eigen;



struct NeedleError : public VectorOfVector {
  ConfigurationPtr cfg0, cfg1;
  double radius;
  KinBodyPtr body;
  NeedleError(ConfigurationPtr cfg0, ConfigurationPtr cfg1, double radius) : cfg0(cfg0), cfg1(cfg1), radius(radius), body(cfg0->GetBodies()[0]) {}
  VectorXd operator()(const VectorXd& a) const {
    cfg0->SetDOFValues(toDblVec(a.topRows(6)));
    OR::Transform Tw0 = body->GetTransform();
    cfg1->SetDOFValues(toDblVec(a.middleRows(6,6)));
    OR::Transform Tw1 = body->GetTransform();
    double theta = a(12);
    OR::Transform Ttarg0(OR::geometry::quatFromAxisAngle(OR::Vector(0,0,1), theta),
        OR::Vector(radius*sin(theta), radius*(1-cos(theta)),0));
        
    OR::Transform Ttargw = Tw0 * Ttarg0;
    OR::Vector position_errA = Ttargw.trans - Tw1.trans;
    OR::Vector ori_err = Ttargw * OR::Vector(1,0,0) - Tw1 * OR::Vector(1,0,0);
    return concat(toVector3d(position_errA), toVector3d(ori_err));

  }
};




struct TrajPlotter1 {
  vector<IncrementalRBPtr> rbs;
  VarArray vars;
  OSGViewerPtr viewer;
  TrajPlotter1(const vector<IncrementalRBPtr>& rbs, const VarArray& vars);
  void OptimizerCallback(OptProb*, DblVec& x);

};

TrajPlotter1::TrajPlotter1(const vector<IncrementalRBPtr>& rbs, const VarArray& vars) : rbs(rbs), vars(vars) {
  viewer = OSGViewer::GetOrCreate(rbs[0]->GetEnv());
}
void TrajPlotter1::OptimizerCallback(OptProb*, DblVec& x) {
  vector<GraphHandlePtr> handles;
  vector<KinBodyPtr> bodies = rbs[0]->GetBodies();
  MatrixXd traj = getTraj(x,vars);
  for (int i=0; i < traj.rows(); ++i) {
    rbs[i]->SetDOFValues(toDblVec(traj.row(i)));
    BOOST_FOREACH(const KinBodyPtr& body, bodies) {
      handles.push_back(viewer->PlotKinBody(body));
      SetTransparency(handles.back(), .35);
    }
  }
  viewer->Idle();
}  



int main(int argc, char** argv)
{
  bool plotting=false, verbose=false;

  {
    Config config;
    config.add(new Parameter<bool>("plotting", &plotting, "plotting"));
    config.add(new Parameter<bool>("verbose", &verbose, "verbose"));
    CommandParser parser(config);
    parser.read(argc, argv);
  }

  RaveInitialize(false, verbose ? Level_Debug : Level_Info);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);
  assert(viewer);

  env->Load(string(DATA_DIR) + "/needleprob.env.xml");
  RobotBasePtr robot = GetRobot(*env);

  int n_steps = 19;
  int n_dof = 6;



  OptProbPtr prob(new OptProb());
  VarArray trajvars;
  AddVarArray(*prob, n_steps, n_dof, "j", trajvars);


  O3Helper helper(robot, trajvars.block(0,3,n_steps,3));
  
  double radius = 1; // turning radius for needle
  VectorXd start(n_dof); start << 0,0,0,0,0,0;
  VectorXd goal(n_dof); goal <<  4,.25,0, 0,0,0;

  VectorXd vel_coeffs = VectorXd::Ones(3);
  prob->addCost(CostPtr(new JointVelCost(trajvars.block(0,0,n_steps, 3), vel_coeffs)));

  helper.AddAngVelCosts(*prob, vel_coeffs[0]);

  double dtheta_lb = (goal.topRows(3) - start.topRows(3)).norm() / (n_steps-1)/radius;
  Var dthetavar = prob->createVariables(singleton<string>("speed"), singleton<double>(dtheta_lb),singleton<double>(INFINITY))[0];

  for (int i=0; i < n_steps-1; ++i) {
    VarVector vars0 = trajvars.row(i), vars1 = trajvars.row(i+1);
    VectorOfVectorPtr f(new NeedleError(helper.m_rbs[i], helper.m_rbs[i+1], radius));
    VectorXd coeffs = VectorXd::Ones(6);
    VarVector vars = concat(vars0, vars1);  
    vars.push_back(dthetavar);
    prob->addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, (boost::format("needle%i")%i).str())));
  }


  for (int j=0; j < n_dof; ++j) {
    prob->addLinearConstraint(exprSub(AffExpr(trajvars(0,j)), start[j]), EQ);
  }
  for (int j=0; j < 3; ++j) { // NO orientation constraint
    prob->addLinearConstraint(exprSub(AffExpr(trajvars(n_steps-1,j)), goal[j]), EQ);
  }
  

  BasicTrustRegionSQP opt(prob);
  helper.ConfigureOptimizer(opt);
  opt.max_iter_ = 500;    

  boost::shared_ptr<TrajPlotter1> plotter;
  if (plotting) {
    plotter.reset(new TrajPlotter1(helper.m_rbs, trajvars));
    opt.addCallback(boost::bind(&TrajPlotter1::OptimizerCallback, boost::ref(plotter), _1, _2));
  }

  MatrixXd initTraj(n_steps, n_dof);  
  for (int idof = 0; idof < n_dof; ++idof) {
    initTraj.col(idof) = VectorXd::LinSpaced(n_steps, start[idof], goal[idof]);
  }
  DblVec initVec = trajToDblVec(initTraj);
  initVec.push_back(dtheta_lb);
  opt.initialize(initVec);
  
  opt.optimize();



  RaveDestroy();


}
