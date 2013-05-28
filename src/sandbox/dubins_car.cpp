#include <openrave-core.h>
#include <openrave/openrave.h>
#include "trajopt/collision_checker.hpp"
#include "utils/stl_to_string.hpp"
#include "trajopt/common.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "osgviewer/osgviewer.hpp"
#include "sco/expr_ops.hpp"
#include <ctime>
#include "utils/eigen_conversions.hpp"
#include "trajopt/trajectory_costs.hpp"
#include "sco/modeling_utils.hpp"
#include "trajopt/plot_callback.hpp"
#include "utils/clock.hpp"
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include "utils/config.hpp"
using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;
using namespace Eigen;
namespace {

bool plotting=false, verbose=false;

#if 0
OR::Transform randomReachablePose(RobotAndDOF& rad, KinBody::LinkPtr link) {
  // todo: save & restore
  DblVec dofvals = rad.RandomDOFValues();
  rad.SetDOFValues(dofvals);
  return link->GetTransform();
}
#endif

}


struct CarError : public VectorOfVector {
  VarVector vars0, vars1;
  Var lvar;
  CarError(const VarVector& vars0, const VarVector& vars1, const Var& lvar) : vars0(vars0), vars1(vars1), lvar(lvar) {}
  VectorXd operator()(const VectorXd& a) const {
    double x0 = a(0), y0 = a(1), t0 = a(2), x1 = a(3), y1 = a(4), t1 = a(5), L = a(6);
    double ang = (t0+t1)/2;
    return Vector2d(x1 - x0 - L*cos(ang), y1 - y0 - L*sin(ang));
  }
};

template <typename T> 
vector<T> singleton(const T& x) {
  return vector<T>(1,x);
} 

int main(int argc, char** argv)
{

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
  env->Load(string(DATA_DIR) + "/carbot.xml");
  OSGViewerPtr viewer;
  if (plotting) {
    viewer.reset(new OSGViewer(env));
    viewer->UpdateSceneData();
    env->AddViewer(viewer);
  }

  RobotBasePtr robot = GetRobot(*env);

  RobotAndDOFPtr rad(new RobotAndDOF(robot, vector<int>(), 11, OR::Vector(0,0,1)));

  int n_steps = 19;
  int n_dof = 3;

  Vector3d start(0,0,0);
  Vector3d goal(0,2,M_PI);

  TrajOptProbPtr prob(new TrajOptProb(n_steps, rad));
  VectorXd vel_coeffs = VectorXd::Ones(3);
  prob->addCost(CostPtr(new JointVelCost(prob->GetVars(), vel_coeffs)));
  
  double lb = (goal - start).norm() / (n_steps-1);
  prob->createVariables(singleton<string>("speed"), singleton<double>(lb),singleton<double>(INFINITY));
  Var lvar = prob->getVars().back();
  
  for (int i=0; i < n_steps-1; ++i) {
    VarVector vars0 = prob->GetVarRow(i), vars1 = prob->GetVarRow(i+1);
    VectorOfVectorPtr f(new CarError(vars0, vars1, lvar));
    VectorXd coeffs = VectorXd::Ones(2);
    VarVector vars = concat(vars0, vars1);  vars.push_back(lvar);
    prob->addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, (boost::format("car%i")%i).str())));
  }

  for (int j=0; j < n_dof; ++j) {
    prob->addLinearConstraint(exprSub(AffExpr(prob->GetVar(0,j)), start[j]), EQ);
  }
  for (int j=0; j < n_dof; ++j) {
    prob->addLinearConstraint(exprSub(AffExpr(prob->GetVar(n_steps-1,j)), goal[j]), EQ);
  }


  BasicTrustRegionSQP opt(prob);
  if (plotting) opt.addCallback(PlotCallback(*prob));
  
  MatrixXd initTraj(n_steps, n_dof);  
  for (int idof = 0; idof < n_dof; ++idof) {
    initTraj.col(idof) = VectorXd::LinSpaced(n_steps, start[idof], goal[idof]);
  }
  DblVec initVec = trajToDblVec(initTraj);
  initVec.push_back(lb);
  opt.initialize(initVec);
    
  opt.optimize();

  RaveDestroy();


}
