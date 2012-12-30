#include "problem_description.hpp"
#include "ipi/logging.hpp"
#include <cstdio>
#include <set>
#include "trajopt/kinematic_constraints.hpp"
#include "ipi/sco/expr_ops.hpp"
#include <boost/foreach.hpp>
using namespace OpenRAVE;
using namespace trajopt;
using Json::Value;
typedef OpenRAVE::EnvironmentBasePtr EnvPtr;


#ifdef __CDT_PARSER__
#define FAIL_IF_FALSE(expr)
#else
#define FAIL_IF_FALSE(expr) if (!expr) {\
    printf("Failure: %s", #expr);\
    return false;\
  }
#endif

namespace {
bool appendVec(Value& v, const Vector3d& x) {
  FAIL_IF_FALSE(v.isArray());
  for (int i=0; i < 3; ++i) v.append(x[i]);
  return true;
}

bool extractVec(const Value& v, VectorXd& out) {
  FAIL_IF_FALSE(v.isArray());
  out.resize(v.size());
  int idx = 0;
  for (Value::const_iterator it = v.begin(); it != v.end(); ++it) {
    out[idx] = (*it).asDouble();
    ++idx;
  }
  return true;
}

template<int n>
bool extract(const Value& v, OR::Vector& out) {
  FAIL_IF_FALSE(v.isArray() && v.size() == n);
  int idx = 0;
  for (Value::iterator it = v.begin(); it != v.end(); ++it) {
    out[idx] = (*it).asDouble();
    ++idx;
  }
  return true;
}

bool extract(const Value& v, Vector3d& out) {
  FAIL_IF_FALSE(v.isArray() && v.size() == 3);
  int idx = 0;
  for (Value::iterator it = v.begin(); it != v.end(); ++it) {
    out[idx] = (*it).asDouble();
    ++idx;
  }
  return true;
}


}

namespace trajopt {





class ProblemBuilder {
  // build TrajOptProblem from request
  // it's a class because it might need to access private data,
  // so these methods are grouped together
public:

  bool DoIt(TrajOptProblemPtr prob, EnvPtr env, TrajOptRequest& req);
  // postconditions: create variables, set m_rad
  bool ReadBasicInfo(const Value& v);
  bool AddCost(const Value& v);
  bool AddConstraint(const Value& v);
  bool Initialize(const Value& v);

private:
  TrajOptProblemPtr m_prob;
  OR::EnvironmentBasePtr m_env;
  RobotAndDOFPtr m_rad;

};

bool ProblemBuilder::DoIt(TrajOptProblemPtr prob, EnvPtr env, TrajOptRequest& req) {

  m_env = env;
  m_prob = prob;

  FAIL_IF_FALSE(req.isMember("basic_info"));
  FAIL_IF_FALSE(ReadBasicInfo(req));


  for (Value::iterator it = req["costs"].begin(); it != req["costs"].end(); ++it) {
    FAIL_IF_FALSE(AddCost(*it));
  }
  for (Value::iterator it = req["costs"].begin(); it != req["costs"].end(); ++it) {
    FAIL_IF_FALSE(AddConstraint(*it));
  }

  FAIL_IF_FALSE(Initialize(req["initialization"]));
  return true;
}

bool ProblemBuilder::ReadBasicInfo(const Value& v) {
  FAIL_IF_FALSE(v.isMember("n_steps"));
  FAIL_IF_FALSE(v.isMember("manip"));

  vector<RobotBasePtr> robots;
  m_env->GetRobots(robots);
  RobotBasePtr robot = robots[0];
  IPI_LOG_WARNING("using first robot in environment");
  RobotBase::ManipulatorPtr manip = GetManipulatorByName(*robot, v["manip"].asString());
  // todo: support various other dof collections that aren't manipulators

  IntVec joint_inds = manip->GetArmIndices();
  m_rad = RobotAndDOFPtr(new RobotAndDOF(robot, joint_inds));

  int n_dof = joint_inds.size();
  int n_steps = v["n_steps"].asInt();

  DblVec lower, upper;
  m_rad->GetDOFLimits(lower, upper);
  vector<double> vlower, vupper;
  vector<string> names;
  for (int i=0; i < n_steps; ++i) {
    vlower.insert(vlower.end(), lower.data(), lower.data()+lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data()+upper.size());
    for (unsigned j=0; j < n_dof; ++j) {
      names.push_back( (boost::format("j_%i_%i")%i%j).str() );
    }
  }

  m_prob->createVariables(names, vlower, vupper);
  m_prob->m_traj_vars = VarArray(n_steps, n_dof, m_prob->vars_.data());
  DblVec cur_dofvals = m_rad->GetDOFValues();


  // fix first timestep vars
  for (int j=0; j < n_dof; ++j) {
    m_prob->addLinearConstr(exprSub(AffExpr(m_prob->m_traj_vars(0,j)), cur_dofvals[j]), EQ);
  }
  return true;

}

bool ProblemBuilder::AddCost(const Value& v) {
  FAIL_IF_FALSE(v.isMember("type"));
  const Value& params = v["params"];
  CostPtr cost;
  string type = v["type"].asString();
  if (type == "point_normal")  {
    Vector3d dir_local, dir_world;
    FAIL_IF_FALSE(extract(params["dir_local"], dir_local));
    FAIL_IF_FALSE(extract(params["dir_world"], dir_world));
    FAIL_IF_FALSE(params.isMember("timestep"));
    KinBody::LinkPtr link = GetLinkMaybeAttached(m_rad->GetRobot(), params["link"].asString());
    FAIL_IF_FALSE(link);
    cost = makeUpCost(m_prob->GetVarRow(m_prob->GetNumSteps()-1), dir_local, dir_world, params["coeff"].asDouble(), m_rad, link);
  }
  else if (type == "pose") {
    OR::Transform T;
    FAIL_IF_FALSE(extract<3>(params["xyz"], T.trans));
    FAIL_IF_FALSE(extract<4>(params["wxyz"], T.rot));

    Vector3d rot_coeffs, pos_coeffs;
    if(params.isMember("pos_coeffs")) {
      FAIL_IF_FALSE(extract(params["pos_coeffs"], pos_coeffs));
    }
    else pos_coeffs.setOnes();
    if(params.isMember("rot_coeffs")) {
      FAIL_IF_FALSE(extract(params["rot_coeffs"], rot_coeffs));
    }
    else rot_coeffs.setOnes();

    KinBody::LinkPtr link = GetLinkMaybeAttached(m_rad->GetRobot(), params["link"].asString());
    FAIL_IF_FALSE(link);
    cost = makeCartPoseCost(m_prob->GetVarRow(m_prob->GetNumSteps()-1), T, rot_coeffs, pos_coeffs, m_rad, link);
  }
  else if (type == "joint_vel") {
    FAIL_IF_FALSE(params.isMember("coeff")));
    cost.reset(new JointVelCost(m_prob, VectorXd::Ones(m_prob->GetNumDOF()) * params["coeff"].asDouble()));
  }
  else {
    IPI_LOG_ERR("unrecognized cost type %s", v["type"].asCString());
    return false;
  }
  m_prob->addCost(cost);
  return true;
}
bool ProblemBuilder::AddConstraint(const Value& v) {
  return false;
}
bool ProblemBuilder::Initialize(const Value& v) {
  return false;
}

TrajOptProblem::TrajOptProblem() {}


TrajOptProblemPtr CreateProblemFromRequest(EnvPtr env, TrajOptRequest& req) {
  TrajOptProblemPtr prob(new TrajOptProblem());
  ::ProblemBuilder builder;
  bool success = builder.DoIt(prob, env, req);
  if (success) return prob;
  else return TrajOptProblemPtr();
}





void HandlePlanningRequest(EnvPtr env, TrajOptRequest& req, TrajOptResponse& resp) {}

}

#undef FAIL_IF_FALSE

