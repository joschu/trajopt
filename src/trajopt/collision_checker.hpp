#pragma once
#include "typedefs.hpp"
#include <set>
#include <utility>
#include <openrave/openrave.h>

namespace trajopt {

class CollisionPairIgnorer {
public:
  void ExcludePair(const KinBody::Link& link1,
                   const KinBody::Link& link2);
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

class CollisionChecker {
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
  virtual void SetContactDistance(float distance) = 0;
  
  ~CollisionChecker() {}
protected:
  CollisionChecker() {}
};
typedef boost::shared_ptr<CollisionChecker> CollisionCheckerPtr;

CollisionCheckerPtr CreateCollisionChecker(OR::EnvironmentBaseConstPtr env);

}

