#include "trajopt/common.hpp"
#include "json_marshal.hpp"
#include <boost/function.hpp>

namespace sco{struct OptResults;}

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
class ObjectState;
typedef boost::shared_ptr<ObjectState> ObjectStatePtr;
class SceneState;
typedef boost::shared_ptr<SceneState> SceneStatePtr;


TrajOptProbPtr TRAJOPT_API ConstructProblem(const ProblemConstructionInfo&);
TrajOptProbPtr TRAJOPT_API ConstructProblem(const Json::Value&, OpenRAVE::EnvironmentBasePtr env);
TrajOptResultPtr TRAJOPT_API OptimizeProblem(TrajOptProbPtr, bool plot);


/**
 * Holds all the data for a trajectory optimization problem
 * so you can modify it programmatically, e.g. add your own costs
 */
class TRAJOPT_API TrajOptProb : public OptProb {
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

  void SetSceneStates(const vector<SceneStatePtr>& s){assert(s.size() == 0 || s.size() == GetNumSteps()); m_scene_states = s;}
  vector<SceneStatePtr>& GetSceneStates() {return m_scene_states;}
  SceneStatePtr GetSceneState(int i) {return m_scene_states.size() == 0 ? SceneStatePtr() : m_scene_states[i];}

  friend TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&);
private:
  VarArray m_traj_vars;
  RobotAndDOFPtr m_rad;
  TrajArray m_init_traj;
  vector<SceneStatePtr> m_scene_states;
  typedef std::pair<string,string> StringPair;
};

struct TRAJOPT_API TrajOptResult {
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

/**
Initialization info read from json
*/
struct InitInfo {
  enum Type {
    STATIONARY,
    GIVEN_TRAJ,
  };
  Type type;
  TrajArray data;
  void fromJson(const Json::Value& v);
};


/**
When cost element of JSON doc is read, one of these guys gets constructed to hold the parameters.
Then it later gets converted to a Cost object by the hatch method
*/
struct TRAJOPT_API CostInfo  {

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

/**
See CostInfo
*/
struct TRAJOPT_API CntInfo  {
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

/**
This object holds all the data that's read from the JSON document
*/
struct TRAJOPT_API ProblemConstructionInfo {
public:
  BasicInfo basic_info;
  vector<CostInfoPtr> cost_infos;
  vector<CntInfoPtr> cnt_infos;
  vector<SceneStatePtr> scene_states;

  InitInfo init_info;

  OR::EnvironmentBasePtr env;
  RobotAndDOFPtr rad;

  ProblemConstructionInfo(OR::EnvironmentBasePtr _env) : env(_env) {}
  void fromJson(const Value& v);

};

/**
 \brief pose error

 See trajopt::PoseCntInfo
 */
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


/**
  \brief Joint space position cost

  \f{align*}{
  \sum_i c_i (x_i - xtarg_i)^2
  \f}
  where \f$i\f$ indexes over dof and \f$c_i\f$ are coeffs
 */
struct JointPosCostInfo : public CostInfo {
  DblVec vals, coeffs;
  int timestep;
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CostInfoPtr create();
};


/**
  \brief Pose constraint

  let \f$T_{err} = T_{targ}^{-1} * T_{cur}\f$.
  Let xerr  be the  translation of \f$T_{err}\f$
  and roterr be the rotation part of the quaternion.
  Then the error vector is [xerr, roterr], scaled by
  pos_coeffs and rot_coeffs, respectively
 */
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

/**
 \brief Motion constraint on link

 Constrains the change in position of the link in each timestep to be less than distance_limit
 */
struct CartVelCntInfo : public CntInfo {
  int first_step, last_step;
  KinBody::LinkPtr link;
  double distance_limit;
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CntInfoPtr create();
};

/**
\brief Joint-space velocity squared

\f{align*}{
  cost = \sum_{t=0}^{T-2} \sum_j c_j (x_{t+1,j} - x_{t,j})^2
\f}
where j indexes over DOF, and \f$c_j\f$ are the coeffs.
*/
struct JointVelCostInfo : public CostInfo {
  DblVec coeffs;
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CostInfoPtr create();
};
/**
\brief %Collision penalty

Distrete-time penalty:
\f{align*}{
  cost = \sum_{t=0}^{T-1} \sum_{A, B} | distpen_t - sd(A,B) |^+
\f}

Continuous-time penalty: same, except you consider swept-out shaps of robot links. Currently self-collisions are not included.

*/
struct CollisionCostInfo : public CostInfo {
  /*
   Note: this is an atypical cost, because you can instantiate it with two different cost types: "collision" and "continuous_collision"
   I should have made "continuous" a parameter, but I'm keeping it this way for backwards compatibility.
   */
  /// first_step and last_step are inclusive
  int first_step, last_step;
  /// coeffs.size() = num_timesteps
  DblVec coeffs;
  /// safety margin: contacts with distance < dist_pen are penalized
  DblVec dist_pen;
  bool continuous;
  CollisionCostInfo(bool _continuous) : continuous(_continuous) {}
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CostInfoPtr create_discrete();
  static CostInfoPtr create_continuous();
};


/**
joint-space position constraint
 */
struct JointConstraintInfo : public CntInfo {
  /// joint values. list of length 1 automatically gets expanded to list of length n_dof
  DblVec vals;
  /// which timestep. default = n_timesteps - 1
  int timestep;
  void fromJson(const Value& v);
  void hatch(TrajOptProb& prob);
  static CntInfoPtr create();
};



}
