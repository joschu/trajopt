#include "rave_utils.hpp"
using namespace OpenRAVE;
#include <boost/foreach.hpp>
#include "utils/logging.hpp"
#include <Eigen/Core>
#include "typedefs.hpp"
using namespace std;
namespace trajopt {

KinBody::LinkPtr GetLinkMaybeAttached(RobotBasePtr robot, const string& name) {
  KinBody::LinkPtr link = robot->GetLink(name);
  if (link) return link;
  std::set<KinBodyPtr> setAttached;
  robot->GetAttached(setAttached);
  BOOST_FOREACH(const KinBodyPtr& body, setAttached) {
    KinBody::LinkPtr link = body->GetLink(name);
    if (link) return link;
  }
  return link;
}

RobotBase::ManipulatorPtr GetManipulatorByName(RobotBase& robot, const std::string& name) {
  vector<RobotBase::ManipulatorPtr> manips = robot.GetManipulators();
  BOOST_FOREACH(RobotBase::ManipulatorPtr& manip, manips) {
    if (manip->GetName()==name) return manip;
  }
  return RobotBase::ManipulatorPtr();
}

RobotBasePtr GetRobotByName(EnvironmentBase& env, const std::string& name) {
  vector<RobotBasePtr> robots;
  env.GetRobots(robots);
  BOOST_FOREACH(RobotBasePtr& robot, robots) {
    if (robot->GetName() == name) return robot;
  }
  return RobotBasePtr();
}

RobotBasePtr GetRobot(EnvironmentBase& env) {
  vector<RobotBasePtr> robots;
  env.GetRobots(robots);
  if (robots.size() == 0) {
    LOG_ERROR("no robots!");
    return RobotBasePtr();
  }
  else if (robots.size() == 1) {
    return robots[0];
  }
  else {
    LOG_ERROR("I don't know which robot you want");
    return RobotBasePtr();
  }
}

int GetRobotLinkIndex(const RobotBase& robot, const KinBody::Link& link) {
  if (link.GetParent().get() == &robot) return link.GetIndex();
  else if (KinBody::LinkPtr grabber = robot.IsGrabbing(link.GetParent())) {
    return grabber->GetIndex();
  }
  else {
    throw OR::openrave_exception("link not part of robot");
  }
}

bool DoesAffect(const RobotBase& robot, const vector<int>& dof_inds, int link_ind) {
  BOOST_FOREACH(const int& dof_ind, dof_inds) {
    if (robot.DoesAffect(dof_ind, link_ind)) return true;
  }
  return false;
}

void PlotAxes(EnvironmentBase& env, const OpenRAVE::Transform& T, float size, vector<GraphHandlePtr>& handles) {
  TransformMatrix mrave = OR::geometry::matrixFromQuat(T.rot);
  DblMatrix m = Eigen::Map<DblMatrix>(mrave.m,3,4);
  Vector x = T.trans + Vector(m(0,0), m(1,0), m(2,0))*size,
         y = T.trans + Vector(m(0,1), m(1,1), m(2,1))*size,
         z = T.trans + Vector(m(0,2), m(1,2), m(2,2))*size;
  handles.push_back(env.drawarrow(T.trans, x, size/10, Vector(1,0,0,1)));
  handles.push_back(env.drawarrow(T.trans, y, size/10, Vector(0,1,0,1)));
  handles.push_back(env.drawarrow(T.trans, z, size/10, Vector(0,0,1,1)));
}

class UserMap : public std::map<std::string, UserDataPtr>, public OpenRAVE::UserData {
};


OR::UserDataPtr GetUserData(const OR::EnvironmentBase& env, const std::string& key) {
  UserDataPtr ud = env.GetUserData();
  if (!ud) return OR::UserDataPtr();
  else if (UserMap* um = dynamic_cast<UserMap*>(ud.get())) {
    UserMap::iterator it = (*um).find(key);
    if (it != (*um).end()) return it->second;
    else return UserDataPtr();
  }
  else {
    throw openrave_exception("someone else is using EnvironmentBase userdata. That screws me up!");
    return OR::UserDataPtr();
  }
}
void SetUserData(OR::EnvironmentBase& env, const std::string& key, OR::UserDataPtr val) {
  UserDataPtr ud = env.GetUserData();
  if (!ud) {
    RAVELOG_INFO("setting userdata\n");
    ud = UserDataPtr(new UserMap());
    env.SetUserData(ud);
  }

  if (UserMap* um = dynamic_cast<UserMap*>(ud.get())) {
    (*um)[key] = val;
  }
  else {
    throw openrave_exception("someone else is using EnvironmentBase userdata. That screws me up!");
  }
}

}
