#include "json_marshal.hpp"
#include <json/json.h>
#include <stdexcept>
using namespace Json;
using namespace std;

namespace json_marshal {

#define IMPLEMENT_READ_PRIMITIVE(T, jsonT, cvtFunc)\
    void fromJson(const Json::Value& v, T& ref) {\
  try {\
    ref = v.cvtFunc();\
  }\
  catch (MY_EXCEPTION&) {\
    throw MY_EXCEPTION( (boost::format("expected: %s, got %s")%(#T)%(v)).str() );\
  }}


IMPLEMENT_READ_PRIMITIVE(bool, boolValue, asBool)
IMPLEMENT_READ_PRIMITIVE(int, intValue, asInt)
IMPLEMENT_READ_PRIMITIVE(double, realValue, asDouble)
IMPLEMENT_READ_PRIMITIVE(string, stringValue, asString)



}
