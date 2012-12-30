#pragma once
#include <json/json.h>
#include <vector>

namespace json_marshal {

bool fromJson(const Json::Value& v, bool& ref);
bool fromJson(const Json::Value& v, int& ref);
bool fromJson(const Json::Value& v, double& ref);
bool fromJson(const Json::Value& v, std::string& ref);
template <class T>
inline bool fromJson(const Json::Value& v, T& ref) {
  return ref.fromJson(v);
}
template <class T>
bool fromJsonArray(const Json::Value& parent, std::vector<T>& ref) {
  bool ok = true;
  ref.clear();
  ref.reserve(parent.size());
  for (Json::Value::const_iterator it = parent.begin(); it != parent.end(); ++it) {
    T t;
    ok &= fromJson(*it, t);
    ref.push_back(t);
  }
  return ok;
}
template <class T>
bool fromJsonArray(const Json::Value& parent, std::vector<T>& ref, int size) {
  if (parent.size() != size) {
    std::cerr << "expected size: " << size << " got: " << parent << std::endl;
    return false;
  }
  else {
    return fromJsonArray(parent, ref);
  }
}
template <class T>
inline bool fromJson(const Json::Value& v, std::vector<T>& ref) {
  return fromJsonArray(v, ref);
}

template <class T>
bool childFromJson(const Json::Value& parent, T& ref, const char* name, const T& df) {
  if (parent.isMember(name)) {
    const Json::Value& v = parent[name];
    return fromJson(v, ref);
  }
  else {
    ref = df;
    return true;
  }
}
template <class T>
bool childFromJson(const Json::Value& parent, T& ref, const char* name) {
  if (parent.isMember(name)) {
    const Json::Value& v = parent[name];
    return fromJson(v, ref);
  }
  else {
    std::cerr << "missing field: " << name << std::endl;
    return false;
  }
}




}
