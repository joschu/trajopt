#include <iostream>
#include "sco/modeling_utils.hpp"
#include "trajopt/robot_and_dof.hpp"
#include "utils/stl_to_string.hpp"
#include "utils/logging.hpp"
#include "sco/expr_op_overloads.hpp"
#include <openrave-core.h>
#include "sco/optimizers.hpp"
#include "trajopt/utils.hpp"
using namespace sco;
using namespace trajopt;
using namespace util;
using namespace OpenRAVE;
using namespace std;

void RunTraj(const TrajArray& traj, EnvironmentBasePtr env, RobotAndDOF& rad) {
  for (int i=0; i < traj.rows(); ++i) {
    {
      EnvironmentMutex::scoped_lock lock(env->GetMutex());
      rad.SetDOFValues(toDblVec(traj.row(i)));
    }
    cin.get();
  }
}

void SetViewer(EnvironmentBasePtr penv, const string& viewername) {
  ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
  BOOST_ASSERT(!!viewer);
  // attach it to the environment:
  penv->Add(viewer);
  // finally call the viewer's infinite loop (this is why a separate thread is needed)
  bool showgui = true;
  viewer->main(showgui);
}

void MakeTrajVariablesAndBounds(int n_steps, float dt, const RobotAndDOF& rad, OptProb& prob_out, VarArray& x, VarArray& v, VarArray& a, VarArray& tq) {
  int n_dof = rad.GetDOF();
  DblVec lower, upper;

  ModelPtr model = prob_out.getModel();
  rad.GetDOFLimits(lower, upper);
  printf("Dof limits: %s, %s\n", Str(lower).c_str(), Str(upper).c_str());
  vector<double> vlower, vupper;
  vector<string> names;
  for (int i=0; i < n_steps; ++i) {
    for (unsigned j=0; j < n_dof; ++j) {
      names.push_back( (boost::format("x_%i_%i")%i%j).str() );
      vlower.push_back(lower[j]);
      vupper.push_back(upper[j]);

      names.push_back( (boost::format("v_%i_%i")%i%j).str() );
      vlower.push_back(-INFINITY);
      vupper.push_back(INFINITY);

      names.push_back( (boost::format("a_%i_%i")%i%j).str() );
      vlower.push_back(-INFINITY);
      vupper.push_back(INFINITY);

      names.push_back( (boost::format("tq_%i_%i")%i%j).str() );
      vlower.push_back(-INFINITY);
      vupper.push_back(INFINITY);
    }
  }
  prob_out.createVariables(names, vlower, vupper);

  cout << Str(names) << endl;

  x.resize(n_steps, n_dof);
  v.resize(n_steps, n_dof);
  a.resize(n_steps, n_dof);
  tq.resize(n_steps, n_dof);

  vector<Var> vars = prob_out.getVars();
  assert(vars.size() == names.size());
  int k = 0;
  for (int i=0; i < n_steps; ++i) {
    for (int j=0; j < n_dof; ++j) {
      x(i,j) = vars[k++];
      v(i,j) = vars[k++];
      a(i,j) = vars[k++];
      tq(i,j) = vars[k++];
    }
  }

  for (int i=0; i < n_steps; ++i) {
    for (int j=0; j < n_dof; ++j) {
//      model->addEqCnt(tq(i,j),"");
      if (i==0) {
        model->addEqCnt(AffExpr(x(i,j)),"");
        model->addEqCnt(AffExpr(v(i,j)),"");
        model->addEqCnt(x(i+1,j) - x(i,j) - dt * v(i,j), "");
        model->addEqCnt(v(i+1,j) - v(i,j) - dt * a(i,j), "");
      }
      else if (i < n_steps-1) {
        model->addEqCnt(x(i+1,j) - x(i,j) - dt * v(i,j), "");
        model->addEqCnt(v(i+1,j) - v(i,j) - dt * a(i,j), "");
      }
      else {
        model->addEqCnt(x(i,j) - x(i-1,j) - dt * v(i,j), "");
        model->addEqCnt(v(i,j) - v(i-1,j) - dt * a(i,j), "");
      }
    }
  }
  model->update();


}

class TorqueCost : public Cost {
  QuadExpr m_quad;
  double m_coeff;
public:
  TorqueCost(const VarArray& tq, double coeff) : m_coeff(coeff) {
    for (int i=0; i < tq.rows(); ++i) {
      for (int j=0; j < tq.cols(); ++j) {
        exprInc(m_quad, m_coeff * exprSquare(tq(i,j)));
      }
    }
  }
  ConvexObjectivePtr convex(const DblVec&, Model* model) {
    ConvexObjectivePtr out(new ConvexObjective(model));
    out->addQuadExpr(m_quad);
    return out;
  }
  double value(const DblVec& x) {
    return m_quad.value(x);
  }
};

struct DynErrCalc {

  RobotAndDOFPtr m_rad;
  VarVector m_x, m_v, m_a, m_tq;
  int m_ndof;
  DynErrCalc(const RobotAndDOFPtr& rad) : m_rad(rad), m_ndof(rad->GetDOF()) {}

  VectorXd operator()(const VectorXd& all) {
    VectorXd x = all.middleRows(0, m_ndof);
    VectorXd v = all.middleRows(m_ndof, m_ndof);
    VectorXd a = all.middleRows(2*m_ndof, m_ndof);
    VectorXd tq = all.middleRows(3*m_ndof, m_ndof);

    m_rad->GetRobot()->SetActiveDOFs(m_rad->GetJointIndices());
    m_rad->GetRobot()->SetActiveDOFValues(toDblVec(x), false);
    m_rad->GetRobot()->SetActiveDOFVelocities(toDblVec(v), false);

    DblVec targ_tq;
    m_rad->GetRobot()->ComputeInverseDynamics(targ_tq, toDblVec(a));
    return toVectorXd(targ_tq) - tq;
  }

};


int main(int argc, char* argv[]) {
  RaveInitialize(true);
  EnvironmentBasePtr env = RaveCreateEnvironment();
//  env->Load("/Users/joschu/Proj/drc/gfe.xml");
  env->Load("/Users/joschu/Desktop/puma.robot.xml");
  vector<RobotBasePtr> robots; env->GetRobots(robots);
  RobotBasePtr gfe = robots[0];
  RobotAndDOFPtr rad(new RobotAndDOF(gfe, gfe->GetManipulators()[0]->GetArmIndices()));

  VarArray x, v, a, tq;
  OptProbPtr prob(new OptProb());
  int n_steps = 10;
  MakeTrajVariablesAndBounds(n_steps, .2/n_steps, *rad, *prob, x, v, a, tq);
  int n_dof = rad->GetDOF();
  printf("n dof: %i\n", n_dof);

  for (int i=0; i < n_steps; ++i) {
    VarVector vars;
    for (int j=0; j < n_dof; ++j) vars.push_back(x(i,j));
    for (int j=0; j < n_dof; ++j) vars.push_back(v(i,j));
    for (int j=0; j < n_dof; ++j) vars.push_back(a(i,j));
    for (int j=0; j < n_dof; ++j) vars.push_back(tq(i,j));
    prob->addCost(CostPtr(new CostFromErrFunc(DynErrCalc(rad), vars, VectorXd::Ones(n_dof), ABS,
        (boost::format("dyn_err%i")%i).str())));
  }

  prob->addCost(CostPtr(new TorqueCost(tq, 1)));
  TrajArray result;
  BasicTrustRegionSQP optimizer(prob);
  optimizer.initialize(DblVec(prob->getVars().size(), 0));
  {
    EnvironmentMutex::scoped_lock lock(env->GetMutex());
    OptStatus status = optimizer.optimize();
  }
  result = getTraj(optimizer.x(), x);

  cout << "torques: " << endl << getTraj(optimizer.x(), tq) << endl;

  boost::thread run_traj(RunTraj, result, env, *rad);

  ViewerBasePtr viewer = RaveCreateViewer(env,"qtosg");
  viewer->main();

}
