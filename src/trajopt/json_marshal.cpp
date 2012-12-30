#include "json_marshal.hpp"
#include <json/json.h>
#include <stdexcept>
using namespace Json;
using namespace std;

namespace json_marshal {

#define IMPLEMENT_READ_PRIMITIVE(T, jsonT, cvtFunc)\
    bool fromJson(const Json::Value& v, T& ref) {\
  try {\
    ref = v.cvtFunc();\
    return true;\
  }\
  catch (std::runtime_error&) {\
    std::cerr << "expected " << #T << ": " << v << endl;\
    return false;\
  }}\


IMPLEMENT_READ_PRIMITIVE(bool, boolValue, asBool)
IMPLEMENT_READ_PRIMITIVE(int, intValue, asInt)
IMPLEMENT_READ_PRIMITIVE(double, realValue, asDouble)
IMPLEMENT_READ_PRIMITIVE(string, stringValue, asString)



}
