#include <json_spirit_reader.h>
#include "trajopt/common.hpp"


namespace trajopt {

typedef json_spirit::Value TrajOptRequest;
typedef json_spirit::Value TrajOptResponse;
using std::string;

class TrajOptProblem {
  /**
   * Holds all the data for a trajectory optimization problem
   * so you can modify it programmatically, e.g. add your own costs
   */
  TrajOptProblem(TrajOptRequest&);
  virtual VarVector GetVarVector(const string& name) = 0;
  virtual VarArray GetVarArray(const string& name) = 0;
  virtual OptProbPtr GetProblem() = 0;
  ~TrajOptProblem() {}
};


void HandlePlanningRequest(TrajOptRequest&, TrajOptResponse&);



}
