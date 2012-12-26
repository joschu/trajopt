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
  int GetNumSteps() {return m_traj_vars.rows();}
  int GetNumDOF() {return m_traj_vars.cols();}
private:
  VarArray m_traj_vars;
};


struct BasicInfo  {
  int n_steps;
  string manip;
  bool fromJson(const Json::Value& v);
};


struct CostInfo  {

  string type;
  string name;
  virtual bool fromJson(const Json::Value& v);

  static CostInfoPtr create(const string& type);
  virtual CostPtr hatch() = 0;
  typedef CostInfoPtr (*MakerFunc)(void);

  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~CostInfo() {}
private:
  static std::map<string, MakerFunc> name2maker;
};

struct CntInfo  {
  string type;
  string name;
  virtual bool fromJson(const Json::Value& v);

  static CntInfoPtr create(const string& type);
  virtual ConstraintPtr hatch() = 0;
  typedef CntInfoPtr (*MakerFunc)(void);

  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~CntInfo() {}
private:
  static std::map<string, MakerFunc> name2maker;
};

bool fromJson(const Json::Value& v, CostInfoPtr&);
bool fromJson(const Json::Value& v, CntInfoPtr&);

class ProblemConstructionInfo {
public:

  BasicInfo basic_info;
  vector<CostInfoPtr> cost_infos;
  vector<CntInfoPtr> cnt_infos;

  bool fromJson(const Value& v);

};



void HandlePlanningRequest(OR::EnvironmentBasePtr env, TrajOptRequest&, TrajOptResponse&);



}
