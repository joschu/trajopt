#pragma once
#include "typedefs.hpp"
#include <set>
#include <utility>
#include <openrave/openrave.h>
#include "robot_and_dof.hpp"

namespace trajopt {

class CollisionPairIgnorer {
public:
  void ExcludePair(const KinBody::Link& link1,
                   const KinBody::Link& link2);
  void AddExcludes(const CollisionPairIgnorer& other);
  bool CanCollide(const KinBody::Link& link1,
                   const KinBody::Link& link2) const;
private:
  typedef std::pair<const KinBody::Link*, const KinBody::Link*> LinkPair;
  std::set<LinkPair> m_pairs;
};

struct Collision {
  const OR::KinBody::Link* linkA;
  const OR::KinBody::Link* linkB;
  OR::Vector ptA, ptB, normalB2A; /* normal points from 2 to 1 */
  double distance; /* pt1 = pt2 + normal*dist */
  float weight, time;
  Collision(const KinBody::Link* linkA, const KinBody::Link* linkB, const OR::Vector& ptA, const OR::Vector& ptB, const OR::Vector& normalB2A, double distance, float weight=1, float time=0) :
    linkA(linkA), linkB(linkB), ptA(ptA), ptB(ptB), normalB2A(normalB2A), distance(distance), weight(weight), time(0) {}
};

class CollisionChecker : public OR::UserData {
public:
  /** 
  Each CollisionChecker object has a copy of the world, so for performance, don't make too many copies  
  */ 
  
  /** CollisionPairIgnorer argument is optional in the following methods */
  
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
  
  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>&) {throw std::runtime_error("not implemented");}

  void ContinuousCheckTrajectory(const TrajArray& traj, RobotAndDOF& rad) {throw std::runtime_error("not implemented");}

  void IgnoreZeroStateSelfCollisions();
  void IgnoreZeroStateSelfCollisions(OpenRAVE::KinBodyPtr body);

  OpenRAVE::EnvironmentBaseConstPtr GetEnv() {return m_env;}
  CollisionPairIgnorer& GetIgnorer() {return m_ignorer;}

  virtual ~CollisionChecker() {}
  static boost::shared_ptr<CollisionChecker> GetOrCreate(OR::EnvironmentBase& env);
protected:
  CollisionChecker(OpenRAVE::EnvironmentBaseConstPtr env) : m_env(env) {}
  OpenRAVE::EnvironmentBaseConstPtr m_env;
  CollisionPairIgnorer m_ignorer;
};
typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;

CollisionCheckerPtr CreateCollisionChecker(OR::EnvironmentBaseConstPtr env);

void PlotCollisions(const std::vector<Collision>& collisions, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles, double safe_dist);

}

