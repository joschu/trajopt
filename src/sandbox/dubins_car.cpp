#include "osgviewer/osgviewer.hpp"
#include "sco/expr_ops.hpp"
#include "sco/modeling_utils.hpp"
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




/**
Let \f$ \bar \theta = (\theta_t + \theta_{t+1})/2 \f$. Then
\f{align*}{
  x_{t+1} - x_t = L(sin(\bar \theta), cos(\bar \theta))
}
*/
struct CarError : public VectorOfVector {
  VectorXd operator()(const VectorXd& a) const {
    double x0 = a(0), y0 = a(1), t0 = a(2), x1 = a(3), y1 = a(4), t1 = a(5), L = a(6);
    double ang = (t0+t1)/2;
    return Vector2d(x1 - x0 - L*cos(ang), y1 - y0 - L*sin(ang));
  }
};



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

  env->Load(string(DATA_DIR) + "/jagged_narrow.xml");
  RobotBasePtr robot = GetRobot(*env);
  RobotAndDOFPtr rad(new RobotAndDOF(robot, vector<int>(), 11, OR::Vector(0,0,1)));




  int n_steps = 29;
  int n_dof = 3;

  Vector3d start(-2,-2,0);
  Vector3d goal(3,2,0);

  OptProbPtr prob(new OptProb());
  VarArray trajvars;
  AddVarArray(*prob, n_steps, n_dof, "j", trajvars);
  
  // penalize dx^2 + dy^2 + dtheta^2
  VectorXd vel_coeffs = VectorXd::Ones(3);
  prob->addCost(CostPtr(new JointVelCost(trajvars, vel_coeffs)));
  
  // lower bound on length per step
  double length_lb = (goal - start).norm() / (n_steps-1);
  // length per step variable
  Var lengthvar = prob->createVariables(singleton<string>("speed"), singleton<double>(length_lb),singleton<double>(INFINITY))[0];



  // Car dynamics constraints
  for (int i=0; i < n_steps-1; ++i) {
    VarVector vars0 = trajvars.row(i), vars1 = trajvars.row(i+1);
    VectorOfVectorPtr f(new CarError());
    VectorXd coeffs = VectorXd::Ones(2);
    VarVector vars = concat(vars0, vars1);  vars.push_back(lengthvar);
    prob->addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, (boost::format("car%i")%i).str())));
    if (i > 0) {
      prob->addCost(CostPtr(new CollisionCost(.01, 10, rad, vars0, vars1)));        
    }
  }

  // start and end constraints
  for (int j=0; j < n_dof; ++j) {
    prob->addLinearConstraint(exprSub(AffExpr(trajvars(0,j)), start[j]), EQ);
  }
  for (int j=0; j < n_dof; ++j) {
    prob->addLinearConstraint(exprSub(AffExpr(trajvars(n_steps-1,j)), goal[j]), EQ);
  }


  // optimization
  BasicTrustRegionSQP opt(prob);
  // if (plotting) opt.addCallback(PlotCallback(*prob));

  // straight line initialization
  MatrixXd initTraj(n_steps, n_dof);  
  for (int idof = 0; idof < n_dof; ++idof) {
    initTraj.col(idof) = VectorXd::LinSpaced(n_steps, start[idof], goal[idof]);
  }
  DblVec initVec = trajToDblVec(initTraj);
  // length variable
  initVec.push_back(length_lb);
  opt.initialize(initVec);
  
  opt.optimize();
  

  RaveDestroy();


}
