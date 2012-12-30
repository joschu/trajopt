#include "trajopt/common.hpp"
#include "json_marshal.hpp"
#include <boost/function.hpp>

namespace trajopt {

using namespace json_marshal;
using namespace Json;
typedef Json::Value TrajOptRequest;
typedef Json::Value TrajOptResponse;
using std::string;

class CostInfo;
typedef boost::shared_ptr<CostInfo> CostInfoPtr;
class CntInfo;
typedef boost::shared_ptr<CntInfo> CntInfoPtr;
class TrajOptProb;
typedef boost::shared_ptr<TrajOptProb> TrajOptProbPtr;
class ProblemConstructionInfo;

TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&);


/**
 * Holds all the data for a trajectory optimization problem
 * so you can modify it programmatically, e.g. add your own costs
 */
class TrajOptProb : public OptProb {
public:
  TrajOptProb();
  ~TrajOptProb() {}
  VarVector GetVarRow(int i) {
    return m_traj_vars.row(i);
  }
  VarArray& GetVars() {
    return m_traj_vars;
  }
  int GetNumSteps() {return m_traj_vars.rows();}
  int GetNumDOF() {return m_traj_vars.cols();}
  RobotAndDOFPtr GetRAD() {return m_rad;}

  friend TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&);
private:
  VarArray m_traj_vars;
  RobotAndDOFPtr m_rad;
};


struct BasicInfo  {
  bool start_fixed;
  int n_steps;
  string manip;
  string robot;
  bool fromJson(const Json::Value& v);
};

struct InitInfo {
  enum Mode {
    GIVEN_TRAJ,
    STATIONARY,
    IK_STRAIGHT_LINE
  };
  Mode mode;
  bool fromJson(const Json::Value& v);
};

struct CostInfo  {

  string type;
  string name;
  virtual bool fromJson(const Json::Value& v);

  static CostInfoPtr fromName(const string& type);
  virtual CostPtr hatch(TrajOptProb& prob) = 0;
  typedef CostInfoPtr (*MakerFunc)();

  /**
   * Registers a user-defined CostInfo so you can use your own cost
   * see function RegisterMakers.cpp
   */
  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~CostInfo() {}
private:
  static std::map<string, MakerFunc> name2maker;
};

struct CntInfo  {
  string type;
  string name;
  virtual bool fromJson(const Json::Value& v);

  static CntInfoPtr fromName(const string& type);
  virtual ConstraintPtr hatch(TrajOptProb& prob) = 0;
  typedef CntInfoPtr (*MakerFunc)();

  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~CntInfo() {}
private:
  static std::map<string, MakerFunc> name2maker;
};

bool fromJson(const Json::Value& v, CostInfoPtr&);
bool fromJson(const Json::Value& v, CntInfoPtr&);

struct ProblemConstructionInfo {
public:
  BasicInfo basic_info;
  vector<CostInfoPtr> cost_infos;
  vector<CntInfoPtr> cnt_infos;
  InitInfo init_info;

  OR::EnvironmentBasePtr env;
  RobotAndDOFPtr rad;

  ProblemConstructionInfo(OR::EnvironmentBasePtr _env) : env(_env) {}
  bool fromJson(const Value& v);

};


struct PoseCostInfo : public CostInfo {
  int timestep;
  Vector3d xyz;
  Vector4d wxyz;
  Vector3d pos_coeffs, rot_coeffs;
  double coeff;
  KinBody::LinkPtr link;
  TrajOptProb* prob;
  bool fromJson(const Value& v);
  CostPtr hatch(TrajOptProb& prob);
  static CostInfoPtr create();
};
struct PositionCostInfo : public CostInfo {
  int timestep;
  Vector3d xyz;
  double coeff;
  TrajOptProb* prob;
  bool fromJson(const Value& v);
  CostPtr hatch(TrajOptProb& prob);
  static CostInfoPtr create();
};
struct JointVelCostInfo : public CostInfo {
  /** cost = coeff * v^2
   * v = dx/dt and dt = (1/T)
   * */
  DblVec coeffs;
  TrajOptProb* prob;
  bool fromJson(const Value& v);
  CostPtr hatch(TrajOptProb& prob);
  static CostInfoPtr create();
};

/**
 * pointer to last TrajOptProb created is stored in a global variable
 * please only use this inside CostInfo::fromJson() or CntInfo::fromJson()
 */
void HandlePlanningRequest(OR::EnvironmentBasePtr env, TrajOptRequest&, TrajOptResponse&);



}
