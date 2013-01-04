#pragma once
#include <json/json.h>
#include <vector>
#include <boost/format.hpp>

class runtime_error_print : public std::runtime_error {
public:
  runtime_error_print(const std::string& s) :
    std::runtime_error(s) {
    std::cerr << "\033[1;31mERROR " << s << "\033[0m\n";
  }
};

#define MY_EXCEPTION runtime_error_print

namespace json_marshal {

void fromJson(const Json::Value& v, bool& ref);
void fromJson(const Json::Value& v, int& ref);
void fromJson(const Json::Value& v, double& ref);
void fromJson(const Json::Value& v, std::string& ref);
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
    std::cerr << "expected size: " << size << " got: " << parent << std::endl;
    throw MY_EXCEPTION("wrong");
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
    throw MY_EXCEPTION((boost::format("missing field: %s")%name).str());
  }
}




}
