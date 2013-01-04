#include "trajopt/collision_checker.hpp"
#include "trajopt/rave_utils.hpp"
#include <boost/foreach.hpp>
using namespace OpenRAVE;

namespace trajopt {


void CollisionPairIgnorer::AddExcludes(const CollisionPairIgnorer& other) {
  m_pairs.insert(other.m_pairs.begin(), other.m_pairs.end());
}

boost::shared_ptr<CollisionChecker> CollisionChecker::GetOrCreate(OR::EnvironmentBase& env) {
  UserDataPtr ud = GetUserData(env, "trajopt_cc");
  if (!ud) {
    RAVELOG_INFO("creating bullet collision checker for environment\n");
    ud =  CreateCollisionChecker(env.shared_from_this());
    SetUserData(env, "trajopt_cc", ud);
  }
  else {
    RAVELOG_INFO("already have a collision checker for this environment\n");
  }
  return boost::dynamic_pointer_cast<CollisionChecker>(ud);
}



void CollisionPairIgnorer::ExcludePair(const KinBody::Link& link1, const KinBody::Link& link2) {

  m_pairs.insert(LinkPair(&link1, &link2));
  m_pairs.insert(LinkPair(&link2, &link1));
}
bool CollisionPairIgnorer::CanCollide(const KinBody::Link& link1, const KinBody::Link& link2) const {
  return m_pairs.find(LinkPair(&link1, &link2)) == m_pairs.end();
}

void CollisionChecker::IgnoreZeroStateSelfCollisions(OpenRAVE::KinBodyPtr body, CollisionPairIgnorer& ignorer) {
  KinBody::KinBodyStateSaver saver(body);
  body->SetDOFValues(DblVec(body->GetDOF(), 0));
  body->SetTransform(Transform(Vector(1,0,0,0), (Vector(-999,-999,-999))));


  vector<Collision> collisions;
  BodyVsAll(*body, &ignorer, collisions);
  RAVELOG_DEBUG("%i extra self collisions in zero state\n", collisions.size());
  for(int i=0; i < collisions.size(); ++i) {
    RAVELOG_DEBUG("ignoring self-collision: %s %s\n", collisions[i].linkA->GetName().c_str(), collisions[i].linkB->GetName().c_str());
    ignorer.ExcludePair(*collisions[i].linkA, *collisions[i].linkB);
  }
}

void CollisionChecker::IgnoreZeroStateSelfCollisions(CollisionPairIgnorer& ignorer) {

  vector<KinBodyPtr> bodies;
  GetEnv()->GetBodies(bodies);

  BOOST_FOREACH(const KinBodyPtr& body, bodies) {
    IgnoreZeroStateSelfCollisions(ignorer);
  }
}



}
