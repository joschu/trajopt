#pragma once
#include <json/json.h>
#include <vector>
#include <boost/format.hpp>
#include <string>
#include <sstream>
#include "trajopt/macros.h"

#define PRINT_AND_THROW(s) do {\
  std::cerr << "\033[1;31mERROR " << s << "\033[0m\n";\
  std::cerr << "at " << __FILE__ << ":" << __LINE__ << std::endl;\
  std::stringstream ss;\
  ss << s;\
  throw std::runtime_error(ss.str());\
} while (0)
#define FAIL_IF_FALSE(expr) if (!expr) {\
    PRINT_AND_THROW( "expected true: " #expr);\
  }


namespace json_marshal {

TRAJOPT_API void fromJson(const Json::Value& v, bool& ref);
TRAJOPT_API void fromJson(const Json::Value& v, int& ref);
TRAJOPT_API void fromJson(const Json::Value& v, double& ref);
TRAJOPT_API void fromJson(const Json::Value& v, std::string& ref);
template <class T>
inline void fromJson(const Json::Value& v, T& ref) {
  ref.fromJson(v);
}
template <class T>
void fromJsonArray(const Json::Value& parent, std::vector<T>& ref) {
  ref.clear();
  ref.reserve(parent.size());
  for (Json::Value::const_iterator it = parent.begin(); it != parent.end(); ++it) {
    T t;
    fromJson(*it, t);
    ref.push_back(t);
  }
}
template <class T>
void fromJsonArray(const Json::Value& parent, std::vector<T>& ref, int size) {
  if (parent.size() != size) {
    PRINT_AND_THROW(boost::format("expected list of size size %i. got: %s\n")%size%parent);
  }
  else {
    fromJsonArray(parent, ref);
  }
}
template <class T>
inline void fromJson(const Json::Value& v, std::vector<T>& ref) {
  fromJsonArray(v, ref);
}

template <class T>
void childFromJson(const Json::Value& parent, T& ref, const char* name, const T& df) {
  if (parent.isMember(name)) {
    const Json::Value& v = parent[name];
    fromJson(v, ref);
  }
  else {
    ref = df;
  }
}
template <class T>
void childFromJson(const Json::Value& parent, T& ref, const char* name) {
  if (parent.isMember(name)) {
    const Json::Value& v = parent[name];
    fromJson(v, ref);
  }
  else {
    PRINT_AND_THROW(boost::format("missing field: %s")%name);
  }
}




}
