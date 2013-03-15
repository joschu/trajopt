#pragma once
#include "macros.h"
#include <openrave/openrave.h>
#include <map>

class UserMap : public std::map<std::string, OpenRAVE::UserDataPtr>, public OpenRAVE::UserData {};

template <typename T>
OpenRAVE::UserDataPtr GetUserData(const T& env, const std::string& key) {
  OpenRAVE::UserDataPtr ud = env.GetUserData();
  if (!ud) return OpenRAVE::UserDataPtr();
  else if (UserMap* um = dynamic_cast<UserMap*>(ud.get())) {
    UserMap::iterator it = (*um).find(key);
    if (it != (*um).end()) return it->second;
    else return OpenRAVE::UserDataPtr();
  }
  else {
    throw OpenRAVE::openrave_exception("Userdata has the wrong class!");
    return OpenRAVE::UserDataPtr();
  }
}
template <typename T>
void SetUserData(T& env, const std::string& key, OpenRAVE::UserDataPtr val) {
  OpenRAVE::UserDataPtr ud = env.GetUserData();
  if (!ud) {
    ud = OpenRAVE::UserDataPtr(new UserMap());
    env.SetUserData(ud);
  }
  if (UserMap* um = dynamic_cast<UserMap*>(ud.get())) {
    (*um)[key] = val;
  }
  else {
    throw OpenRAVE::openrave_exception("userdata has unexpected class");
  }
}
template <typename T>
void RemoveUserData(T& body, const std::string& key) {
  if (UserMap* um = dynamic_cast<UserMap*>(GetUserData(body, key).get())) {
    (*um).erase(key);
  }
}

