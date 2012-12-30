#include "trajopt/problem_description.hpp"
#include "trajopt/common.hpp"
#include <boost/foreach.hpp>
#include "ipi/logging.hpp"
#include "ipi/sco/expr_ops.hpp"
#include "trajopt/kinematic_constraints.hpp"
#include "trajopt/rave_utils.hpp"
using namespace Json;
using namespace std;
using namespace OpenRAVE;
using namespace trajopt;


#ifdef __CDT_PARSER__
#define FAIL_IF_FALSE(expr)
#else
#define FAIL_IF_FALSE(expr) if (!expr) {\
    printf("Failure: %s\n", #expr);\
    return false;\
  }
#endif

namespace {


bool gRegisteredMakers;
void RegisterMakers() {
  CostInfo::RegisterMaker("pose", &PoseCostInfo::create);
  CostInfo::RegisterMaker("joint_vel", &JointVelCostInfo::create);
}


}

namespace Json {

bool fromJson(const Json::Value& v, Vector3d& x) {
  vector<double> vx;
  if (fromJsonArray(v, vx, 3)) {
    x = Vector3d(vx[0], vx[1], vx[2]);
    return true;
  }
  else return false;
}
bool fromJson(const Json::Value& v, Vector4d& x) {
  vector<double> vx;
  if (fromJsonArray(v, vx, 4)) {
    x = Vector4d(vx[0], vx[1], vx[2], vx[3]);
    return true;
  }
  else return false;
}

}

namespace trajopt {

ProblemConstructionInfo* gPCI;

bool BasicInfo::fromJson(const Json::Value& v) {
  bool ok = true;
  ok &= childFromJson(v, start_fixed, "start_fixed", true);
  ok &= childFromJson(v, n_steps, "n_steps");
  ok &= childFromJson(v, manip, "manip");
  ok &= childFromJson(v, robot, "robot", string(""));
  return ok;
}


////
bool CostInfo::fromJson(const Json::Value& v) {
  childFromJson(v, type, "type");
  childFromJson(v, type, "name", string("unnamed"));
  return true;
}
bool fromJson(const Json::Value& v, CostInfoPtr& cost) {
  string type;
  if (!childFromJson(v, type, "type")) return false;
  cost = CostInfo::fromName(type);
  return cost && cost->fromJson(v);
}
CostInfoPtr CostInfo::fromName(const string& type) {
  if (!gRegisteredMakers) RegisterMakers();
  if (name2maker.find(type) != name2maker.end()) {
    return (*name2maker[type])();
  }
  else return CostInfoPtr();
}
map<string, CostInfo::MakerFunc> CostInfo::name2maker;

void CostInfo::RegisterMaker(const std::string& type, MakerFunc f) {
  name2maker[type] = f;
}


////
//// almost copied
bool CntInfo::fromJson(const Json::Value& v) {
  childFromJson(v, type, "type");
  childFromJson(v, type, "name", string("unnamed"));
  return true;
}
#define FAIL_IF_EXCEPTION(expr)\
  try(expr) catch(std::runtime_error&) return false;

bool fromJson(const Json::Value& v, CntInfoPtr& cnt) {
  string type;
  if (!childFromJson(v, type, "type")) return false;
  cnt = CntInfo::fromName(type);
  return cnt && cnt->fromJson(v);
}
CntInfoPtr CntInfo::fromName(const string& type) {
  if (!gRegisteredMakers) RegisterMakers();
  if (name2maker.find(type) != name2maker.end()) {
    return (*name2maker[type])();
  }
  else {
    return CntInfoPtr();
  }
}
map<string,CntInfo::MakerFunc> CntInfo::name2maker;

bool ProblemConstructionInfo::fromJson(const Value& v) {
  bool ok = true;
  ok &= childFromJson(v, basic_info, "basic_info");

  RobotBasePtr robot = (basic_info.robot=="") ? GetRobot(*env) : GetRobotByName(*env, basic_info.robot);
  if (!robot) {
    IPI_LOG_ERR("couldn't get robot");
    return false;
  }
  RobotBase::ManipulatorPtr manip = GetManipulatorByName(*robot, basic_info.manip);
  if (!manip) {
    IPI_LOG_ERR("couldn't get manip %s", basic_info.manip);
    return false;
  }
  rad = RobotAndDOFPtr(new RobotAndDOF(robot, manip->GetArmIndices()));

  gPCI = this;
  if (v.isMember("costs")) ok &= fromJsonArray(v["costs"], cost_infos);
  if (v.isMember("constraints")) ok &= fromJsonArray(v["constraints"], cnt_infos);
  gPCI = NULL;

  return ok;
}

TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo& pci) {
  TrajOptProbPtr prob(new TrajOptProb());

  const BasicInfo& bi = pci.basic_info;
  int n_steps = bi.n_steps;

  prob->m_rad = pci.rad;
  // todo: support various other dof collections that aren't manipulators
  int n_dof = prob->m_rad->GetDOF();


  DblVec lower, upper;
  prob->m_rad->GetDOFLimits(lower, upper);
  vector<double> vlower, vupper;
  vector<string> names;
  for (int i=0; i < n_steps; ++i) {
    vlower.insert(vlower.end(), lower.data(), lower.data()+lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data()+upper.size());
    for (unsigned j=0; j < n_dof; ++j) {
      names.push_back( (boost::format("j_%i_%i")%i%j).str() );
    }
  }
  prob->createVariables(names, vlower, vupper);
  prob->m_traj_vars = VarArray(n_steps, n_dof, prob->vars_.data());

  DblVec cur_dofvals = prob->m_rad->GetDOFValues();

  if (bi.start_fixed) {
    for (int j=0; j < n_dof; ++j) {
      prob->addLinearConstr(exprSub(AffExpr(prob->m_traj_vars(0,j)), cur_dofvals[j]), EQ);
    }
  }


  BOOST_FOREACH(const CostInfoPtr& ci, pci.cost_infos) {
    CostPtr cost = ci->hatch(*prob);
    prob->addCost(cost);
  }
  BOOST_FOREACH(const CntInfoPtr& ci, pci.cnt_infos) {
    ConstraintPtr cnt = ci->hatch(*prob);
    prob->addConstr(cnt);
  }

  return prob;

}


TrajOptProb::TrajOptProb() {
}


bool PoseCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  bool ok = true;
  ok &= childFromJson(params, timestep, "timestep", gPCI->basic_info.n_steps-1);
  ok &= childFromJson(params, xyz,"xyz");
  ok &= childFromJson(params, wxyz,"wxyz");
  ok &= childFromJson(params, pos_coeffs,"pos_coeffs", (Vector3d)Vector3d::Ones());
  ok &= childFromJson(params, rot_coeffs,"rot_coeffs", (Vector3d)Vector3d::Ones());

  string linkstr;
  ok &= childFromJson(params, linkstr, "link");
  link = gPCI->rad->GetRobot()->GetLink(linkstr);
  if (!link) {
    cerr << "invalid link name " << linkstr << endl;
    return false;
  }
  return ok;
}
CostInfoPtr PoseCostInfo::create() {
  CostInfoPtr out(new PoseCostInfo());
  return out;
}
CostPtr PoseCostInfo::hatch(TrajOptProb& prob) {
  return CostPtr(new CartPoseCost(prob.GetVarRow(timestep), toRaveTransform(wxyz, xyz), rot_coeffs, pos_coeffs, prob.GetRAD(), link));
}

bool JointVelCostInfo::fromJson(const Value& v) {
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  bool ok = true;

  ok &= childFromJson(params, coeffs,"coeffs");
  int n_dof = gPCI->rad->GetDOF();
  if (coeffs.size() == 1) coeffs = DblVec(n_dof, coeffs[0]);
  else if (coeffs.size() != n_dof) {
    IPI_LOG_ERR("wrong number of coeffs. expected %i got %i", n_dof, coeffs.size());
  }

  return ok;
}
CostInfoPtr JointVelCostInfo::create() {
  CostInfoPtr out(new JointVelCostInfo());
  return out;
}
CostPtr JointVelCostInfo::hatch(TrajOptProb& prob) {
  return CostPtr(new JointVelCost(prob.GetVars(), Eigen::Map<VectorXd>(coeffs.data(), coeffs.size())));
}





}

