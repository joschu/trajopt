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
  float weight;
  Collision(const KinBody::Link* linkA, const KinBody::Link* linkB, const OR::Vector& ptA, const OR::Vector& ptB, const OR::Vector& normalB2A, double distance, float weight=1) :
    linkA(linkA), linkB(linkB), ptA(ptA), ptB(ptB), normalB2A(normalB2A), distance(distance), weight(weight) {}
};

class CollisionChecker : public OR::UserData {
public:
  /** 
  Each CollisionChecker object has a copy of the world, so for performance, don't make too many copies  
  */ 
  
  /** CollisionPairIgnorer argument is optional in the following methods */
  
  /** check everything vs everything else */
  virtual void AllVsAll(const CollisionPairIgnorer* ignorer, vector<Collision>& collisions)=0;
  /** check link vs everything else */
  virtual void LinkVsAll(const KinBody::Link& link, const CollisionPairIgnorer* ignorer, vector<Collision>& collisions)=0;
  /** check robot vs everything else. includes attached bodies */
  virtual void BodyVsAll(const KinBody& body, const CollisionPairIgnorer* ignorer, vector<Collision>& collisions)=0;
  /** contacts of distance < (arg) will be returned */
  virtual void SetContactDistance(float distance) {m_contactDistance = distance;};
  virtual float GetContactDistance() {return m_contactDistance;};
  
  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>&) {throw std::runtime_error("not implemented");}

  void ContinuousCheckTrajectory(const TrajArray& traj, const RobotBase& rad) {throw std::runtime_error("not implemented");}

  void IgnoreZeroStateSelfCollisions(CollisionPairIgnorer& ignorer);
  void IgnoreZeroStateSelfCollisions(OpenRAVE::KinBodyPtr body, CollisionPairIgnorer& ignorer);
  OpenRAVE::EnvironmentBaseConstPtr GetEnv() {return m_env;}

  virtual ~CollisionChecker() {}
  static boost::shared_ptr<CollisionChecker> GetOrCreate(OR::EnvironmentBase& env);
protected:
  CollisionChecker(OpenRAVE::EnvironmentBaseConstPtr env) : m_env(env), m_contactDistance(.02) {}
  double m_contactDistance;
  OpenRAVE::EnvironmentBaseConstPtr m_env;
};
typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;

CollisionCheckerPtr CreateCollisionChecker(OR::EnvironmentBaseConstPtr env);

void PlotCollisions(const std::vector<Collision>& collisions, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles, double safe_dist);

}

