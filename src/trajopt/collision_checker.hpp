#pragma once
#include "typedefs.hpp"
#include <set>
#include <utility>
#include <iostream>
#include <openrave/openrave.h>
#include "robot_and_dof.hpp"
#include "macros.h"

namespace trajopt {

struct Collision {
  const OR::KinBody::Link* linkA;
  const OR::KinBody::Link* linkB;
  OR::Vector ptA, ptB, normalB2A; /* normal points from 2 to 1 */
  double distance; /* pt1 = pt2 + normal*dist */
  float weight, time;
  Collision(const KinBody::Link* linkA, const KinBody::Link* linkB, const OR::Vector& ptA, const OR::Vector& ptB, const OR::Vector& normalB2A, double distance, float weight=1, float time=0) :
    linkA(linkA), linkB(linkB), ptA(ptA), ptB(ptB), normalB2A(normalB2A), distance(distance), weight(weight), time(0) {}
};
TRAJOPT_API std::ostream& operator<<(std::ostream&, const Collision&);


/** 
Each CollisionChecker object has a copy of the world, so for performance, don't make too many copies  
*/ 
class TRAJOPT_API CollisionChecker : public OR::UserData {
public:

  /** check everything vs everything else */
  virtual void AllVsAll(vector<Collision>& collisions)=0;
  /** check link vs everything else */
  virtual void LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions)=0;
  virtual void LinksVsAll(const vector<KinBody::LinkPtr>& links, vector<Collision>& collisions)=0;

  /** check robot vs everything else. includes attached bodies */
  void BodyVsAll(const KinBody& body, vector<Collision>& collisions) {
    LinksVsAll(body.GetLinks(), collisions);
  }
  /** contacts of distance < (arg) will be returned */
  virtual void SetContactDistance(float distance)  = 0;
  virtual double GetContactDistance() = 0;
  
  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>&) {throw std::runtime_error("not implemented");}

  virtual void ContinuousCheckTrajectory(const TrajArray& traj, RobotAndDOF& rad, vector<Collision>& collisions) {throw std::runtime_error("not implemented");}
  virtual void CastVsAll(RobotAndDOF& rad, const vector<KinBody::LinkPtr>& links, const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions) {throw std::runtime_error("not implemented");}

  void IgnoreZeroStateSelfCollisions();
  void IgnoreZeroStateSelfCollisions(OpenRAVE::KinBodyPtr body);

  virtual void ExcludeCollisionPair(const KinBody::Link& link0, const KinBody::Link& link1) = 0;


  OpenRAVE::EnvironmentBaseConstPtr GetEnv() {return m_env;}

  virtual ~CollisionChecker() {}
  static boost::shared_ptr<CollisionChecker> GetOrCreate(OR::EnvironmentBase& env);
protected:
  CollisionChecker(OpenRAVE::EnvironmentBaseConstPtr env) : m_env(env) {}
  OpenRAVE::EnvironmentBaseConstPtr m_env;
};
typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;

CollisionCheckerPtr TRAJOPT_API CreateCollisionChecker(OR::EnvironmentBaseConstPtr env);

TRAJOPT_API void PlotCollisions(const std::vector<Collision>& collisions, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles, double safe_dist);

}

