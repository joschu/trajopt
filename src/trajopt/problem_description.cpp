#include "trajopt/problem_description.hpp"
using namespace Json;
using namespace std;

namespace trajopt {

bool BasicInfo::fromJson(const Json::Value& v) {
  bool ok = true;
  ok &= childFromJson(v, n_steps, "n_steps");
  ok &= childFromJson(v, manip, "manip");
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
  cost = CostInfo::create(type);
  return cost && cost->fromJson(v);
}
CostInfoPtr CostInfo::create(const string& type) {
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
  cnt = CntInfo::create(type);
  return cnt && cnt->fromJson(v);
}
CntInfoPtr CntInfo::create(const string& type) {
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
  if (v.isMember("costs")) ok &= fromJsonArray(v["costs"], cost_infos);
  if (v.isMember("constraints")) ok &= fromJsonArray(v["constraints"], cnt_infos);
  return ok;
}





}

