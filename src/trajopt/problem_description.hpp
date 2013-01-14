#include "trajopt/common.hpp"
#include "json_marshal.hpp"
#include <boost/function.hpp>

namespace ipi{namespace sco{struct OptResults;}}

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
class TrajOptResult;
typedef boost::shared_ptr<TrajOptResult> TrajOptResultPtr;


TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&);
TrajOptProbPtr ConstructProblem(const Json::Value&, OpenRAVE::EnvironmentBasePtr env);
TrajOptResultPtr OptimizeProblem(TrajOptProbPtr, bool plot);
void HandlePlanningRequest(OR::EnvironmentBasePtr env, TrajOptRequest&, TrajOptResponse&);




/**
 * Holds all the data for a trajectory optimization problem
 * so you can modify it programmatically, e.g. add your own costs
 */
class TrajOptProb : public OptProb {
public:
  TrajOptProb();
  TrajOptProb(int n_steps, RobotAndDOFPtr rad);
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
  OR::EnvironmentBasePtr GetEnv() {return m_rad->GetRobot()->GetEnv();}

  void SetInitTraj(const TrajArray& x) {m_init_traj = x;}
  TrajArray GetInitTraj() {return m_init_traj;}

  friend TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&);
private:
  VarArray m_traj_vars;
  RobotAndDOFPtr m_rad;
  TrajArray m_init_traj;
  typedef std::pair<string,string> StringPair;
};

struct TrajOptResult {
  vector<string> cost_names, cnt_names;
  vector<double> cost_vals, cnt_viols;
  TrajArray traj;
  TrajOptResult(OptResults& opt, TrajOptProb& prob);
};


struct BasicInfo  {
  bool start_fixed;
  int n_steps;
  string manip;
  string robot; // optional
  IntVec dofs_fixed; // optional
  void fromJson(const Json::Value& v);
};

struct InitInfo {
  enum Type {
    STATIONARY,
    GIVEN_TRAJ,
  };
  Type type;
  TrajArray data;
  void fromJson(const Json::Value& v);
};

struct CostInfo  {

  string name;
  virtual void fromJson(const Json::Value& v)=0;

  static CostInfoPtr fromName(const string& type);
  virtual void hatch(TrajOptProb& prob) = 0;
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
  string name;
  virtual void fromJson(const Json::Value& v) = 0;

  static CntInfoPtr fromName(const string& type);
  virtual void hatch(TrajOptProb& prob) = 0;
  typedef CntInfoPtr (*MakerFunc)();

  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~CntInfo() {}
private:
  static std::map<string, MakerFunc> name2maker;
};

void fromJson(const Json::Value& v, CostInfoPtr&);
void fromJson(const Json::Value& v, CntInfoPtr&);

struct ProblemConstructionInfo {
public:
  BasicInfo basic_info;
  vector<CostInfoPtr> cost_infos;
  vector<CntInfoPtr> cnt_infos;
  InitInfo init_info;

  OR::EnvironmentBasePtr env;
  RobotAndDOFPtr rad;

  ProblemConstructionInfo(OR::EnvironmentBasePtr _env) : env(_env) {}
  void fromJson(const Value& v);

};


struct PoseCostInfo : public CostInfo {
  int timestep;
  Vector3d xyz;
  Vector4d wxyz;
  Vector3d pos_coeffs, rot_coeffs;
  double coeff;
  KinBody::LinkPtr link;
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CostInfoPtr create();
};
struct PoseCntInfo : public CntInfo {
  int timestep;
  Vector3d xyz;
  Vector4d wxyz;
  Vector3d pos_coeffs, rot_coeffs;
  double coeff;
  KinBody::LinkPtr link;
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CntInfoPtr create();
};

struct JointVelCostInfo : public CostInfo {
  /** cost = coeff * v^2
   * v = dx/dt and dt = (1/T)
   * */
  DblVec coeffs;
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CostInfoPtr create();
};
struct CollisionCostInfo : public CostInfo {
  DblVec coeffs, dist_pen;
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CostInfoPtr create();
};


struct JointConstraintInfo : public CntInfo {
  DblVec vals;
  int timestep;
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CntInfoPtr create();
};



}
