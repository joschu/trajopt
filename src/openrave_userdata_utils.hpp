#pragma once
#include "macros.h"
#include <openrave/openrave.h>
#include <map>
#include "utils/logging.hpp"

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

class UserMap : public std::map<std::string, OpenRAVE::UserDataPtr>, public OpenRAVE::UserData {};

namespace trajopt {

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
  OpenRAVE::UserDataPtr ud = body.GetUserData();
  if (UserMap* um = dynamic_cast<UserMap*>(ud.get())) {
    if (um->find(key) == um->end()) LOG_WARN("tried to erase key %s but it's not in the userdata map!", key.c_str());
    (*um).erase(key);
  }
  else {
    LOG_ERROR("body %s has no userdata map", body.GetName().c_str());
  }
}


}
