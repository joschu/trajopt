#include "trajopt/collision_checker.hpp"
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <openrave-core.h>
#include <boost/foreach.hpp>
#include <vector>

using namespace std;
using namespace trajopt;

namespace {


ostream &operator<<(ostream &stream, const btVector3& v) {
  stream << v.x() << " " << v.y() << " " << v.z();
  return stream;
}

class CollisionObjectWrapper : public btCollisionObject, OR::UserData {
public:
  CollisionObjectWrapper(KinBody::Link* link) : m_link(link) {}
  vector<boost::shared_ptr<void> > m_data;
  KinBody::Link* m_link;
  template<class T>
  void manage(T* t) { // manage memory of this object
    m_data.push_back(boost::shared_ptr<T>(t));
  }
};
typedef boost::shared_ptr<CollisionObjectWrapper> COWPtr;

const CollisionPairIgnorer* gPairIgnorer;

void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {
  if ( gPairIgnorer==NULL || gPairIgnorer->CanCollide(*(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy0->m_clientObject)->m_link),
      *(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy1->m_clientObject)->m_link)));
  dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}


btVector3 toBt(const OR::Vector& v){
  return btVector3(v[0], v[1], v[2]);
}
OR::Vector toOR(const btVector3& v) {
  return OR::Vector(v.x(), v.y(), v.z());
}
btQuaternion toBtQuat(const OR::Vector& q) {
  return btQuaternion(q[1], q[2], q[3], q[0]);
}
btTransform toBt(const OR::Transform& t){
  return btTransform(toBtQuat(t.rot), toBt(t.trans));
}

COWPtr CollisionObjectFromLink(KinBody::LinkPtr link) {
  RAVELOG_DEBUG("creating bt collision object from from %s",link->GetName().c_str());

  //#if OPENRAVE_VERSION_MINOR>6
  const std::vector<boost::shared_ptr<OpenRAVE::KinBody::Link::GEOMPROPERTIES> > & geometries=link->GetGeometries();
  //#else
  //  const std::list<KinBody::Link::GEOMPROPERTIES> &geometries =link->GetGeometries();
  //#endif
  // sometimes the OpenRAVE link might not even have any geometry data associated with it
  // (this is the case with the PR2 model). therefore just add an empty BulletObject
  // pointer so we know to skip it in the future
  if (geometries.empty()) return COWPtr();

  //	bool useCompound = geometries.size() > 1;

  COWPtr cow(new CollisionObjectWrapper(link.get()));

  btCompoundShape* compound = new btCompoundShape();
  cow->manage(compound);
  compound->setMargin(0); //margin: compound. seems to have no effect when positive but has an effect when negative
  cow->setCollisionShape(compound);


  //#if OPENRAVE_VERSION_MINOR>6
  BOOST_FOREACH(const boost::shared_ptr<OpenRAVE::KinBody::Link::GEOMPROPERTIES>& geom, geometries) {
    //#else
    //    for (std::list<KinBody::Link::GEOMPROPERTIES>::const_iterator geom = geometries.begin(); geom != geometries.end(); ++geom) {
    //#endif

    const KinBody::Link::TRIMESH &mesh = geom->GetCollisionMesh();
    btCollisionShape* subshape;

    switch (geom->GetType()) {
    case KinBody::Link::GEOMPROPERTIES::GeomBox:
      subshape = new btBoxShape(toBt(geom->GetBoxExtents()));
      break;
    case KinBody::Link::GEOMPROPERTIES::GeomSphere:
      subshape = new btSphereShape(geom->GetSphereRadius());
      break;
    case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
      // cylinder axis aligned to Y
    {
      float r=geom->GetCylinderRadius(), h = geom->GetCylinderHeight()/2;
      subshape = new btCylinderShapeZ(btVector3(r,r,h/2));
      break;
    }
    case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
    {
      assert(mesh.indices.size() >= 3);
      btTriangleMesh* ptrimesh = new btTriangleMesh();

      for (size_t i = 0; i < mesh.indices.size(); i += 3)
        ptrimesh->addTriangle(
            toBt(mesh.vertices[i]),
            toBt(mesh.vertices[i+1]),
            toBt(mesh.vertices[i+2]));
      // store the trimesh somewhere so it doesn't get deallocated by the smart pointer
      cow->manage(ptrimesh);

      btConvexTriangleMeshShape convexTrimesh(ptrimesh);
      convexTrimesh.setMargin(0); // margin: hull padding

      //Create a hull shape to approximate Trimesh
      btShapeHull shapeHull(&convexTrimesh);
      shapeHull.buildHull(-666); // note: margin argument not used

      btConvexHullShape *convexShape = new btConvexHullShape();
      cow->manage(convexShape);
      for (int i = 0; i < shapeHull.numVertices(); ++i)
        convexShape->addPoint(shapeHull.getVertexPointer()[i]);

      subshape = convexShape;
      break;
    }
    default:
      assert(0 && "unrecognized collision shape type");
    }

    assert(subshape != NULL);
    cow->manage(subshape);
    subshape->setMargin(0);
    btTransform geomTrans = toBt(geom->GetTransform());
    compound->addChildShape(geomTrans, subshape);

  }

  cow->setWorldTransform(toBt(link->GetTransform()));

  return cow;
}


class BulletCollisionChecker : public CollisionChecker {
  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;
  OR::EnvironmentBaseConstPtr m_env;
  typedef map<OR::KinBody::LinkPtr, COWPtr> Link2Cow;
  Link2Cow m_link2cow;

public:
  BulletCollisionChecker(OR::EnvironmentBaseConstPtr env) {
    m_env = env;
    m_coll_config = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_coll_config);
    m_broadphase = new btDbvtBroadphase();
    m_world = new btCollisionWorld(m_dispatcher, m_broadphase, m_coll_config);
    m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,
        m_coll_config->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));
  }

  ~BulletCollisionChecker() {
    delete m_world;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_coll_config;
  }

  void SetContactDistance(float expansion) {
    SHAPE_EXPANSION = btVector3(1,1,1)*expansion;
    gContactBreakingThreshold = 2*expansion;
    btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
    if (objs.size() == 0) RAVELOG_WARN("call SetBulletContactDistance AFTER adding objects to the world!\n");
    for (int i=0; i < objs.size(); ++i) {
      objs[i]->setContactProcessingThreshold(expansion);
    }
    btCollisionDispatcher* dispatcher = static_cast<btCollisionDispatcher*>(m_world->getDispatcher());
    dispatcher->setDispatcherFlags(dispatcher->getDispatcherFlags() & ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
  }

  void AllVsAll(const CollisionPairIgnorer* ignorer, vector<Collision>& collisions) {
    UpdateBulletFromRave();
    gPairIgnorer = ignorer;


    RAVELOG_DEBUG("starting collision check\n");
    m_world->performDiscreteCollisionDetection();
    int numManifolds = m_dispatcher->getNumManifolds();
    RAVELOG_DEBUG("number of manifolds: %i\n", numManifolds);
    for (int i = 0; i < numManifolds; ++i) {
      btPersistentManifold* contactManifold = m_dispatcher->getManifoldByIndexInternal(i);
      int numContacts = contactManifold->getNumContacts();
      RAVELOG_DEBUG("number of contacts in manifold %i: %i\n", i, numContacts);
      const CollisionObjectWrapper* objA = static_cast<const CollisionObjectWrapper*>(contactManifold->getBody0());
      const CollisionObjectWrapper* objB = static_cast<const CollisionObjectWrapper*>(contactManifold->getBody1());
      for (int j = 0; j < numContacts; ++j) {
        btManifoldPoint& pt = contactManifold->getContactPoint(j);
        // adjustContactPoint(pt, objA, objB);

        const KinBody::Link* bodyA = objA->m_link;
        const KinBody::Link* bodyB = objB->m_link;

        if (!ignorer || ignorer->CanCollide(*bodyA, *bodyB)) collisions.push_back(Collision(bodyA, bodyB, toOR(pt.m_positionWorldOnA), toOR(pt.m_positionWorldOnB),
            toOR(pt.m_normalWorldOnB), pt.m_distance1, 1./numContacts));
        else {
          RAVELOG_DEBUG("ignoring collision between %s and %s\n", bodyA->GetName().c_str(), bodyB->GetName().c_str());
          assert(0 && "this shouldn't happen because we're filtering at narrowphase");
        }
        RAVELOG_DEBUG("%s - %s collided\n", bodyA->GetName().c_str(), bodyB->GetName().c_str());
      }
      // caching helps performance, but for optimization the cost should not be history-dependent
      contactManifold->clearManifold();
    }
  }

  virtual void UpdateBulletFromRave() {
    
    //
    vector<OR::KinBodyPtr> bodies;
    m_env->GetBodies(bodies);
    RAVELOG_DEBUG("%i kinbodies in rave env\n", bodies.size());
    BOOST_FOREACH(const OR::KinBodyPtr& body, bodies) {
      BOOST_FOREACH(const OR::KinBody::LinkPtr& link, body->GetLinks()) {

        Link2Cow::iterator it = m_link2cow.find(link);
        if (it == m_link2cow.end()) {
          COWPtr new_cow = CollisionObjectFromLink(link);
          if (new_cow) {
            m_link2cow[link] = new_cow;
            assert(new_cow);
            m_world->addCollisionObject(new_cow.get());
            RAVELOG_DEBUG("added collision object for  link %s\n", link->GetName().c_str());
          }
          else {
            RAVELOG_DEBUG("ignoring link %s", link->GetName().c_str());
          }
        }
      }
    }
    
    RAVELOG_WARN("todo: remove objects\n");

    btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
    RAVELOG_DEBUG("%i objects in bullet world\n", objs.size());
    for (int i=0; i < objs.size(); ++i) {
      CollisionObjectWrapper* cow = static_cast<CollisionObjectWrapper*>(objs[i]);
      cow->setWorldTransform(toBt(cow->m_link->GetTransform()));
    }
  }

  virtual void LinkVsAll(const KinBody::Link& link, const CollisionPairIgnorer* ignorer, vector<Collision>& collisions) {
    throw OR::openrave_exception("method not implemented", OR::ORE_NotImplemented);
  }
  virtual void BodyVsAll(const KinBody& body, const CollisionPairIgnorer* ignorer, vector<Collision>& collisions) {
    throw OR::openrave_exception("method not implemented", OR::ORE_NotImplemented);
  }



};



}







namespace trajopt {


void CollisionPairIgnorer::ExcludePair(const KinBody::Link& link1, const KinBody::Link& link2) {
  m_pairs.insert(LinkPair(&link1, &link2));
  m_pairs.insert(LinkPair(&link2, &link1));
}
bool CollisionPairIgnorer::CanCollide(const KinBody::Link& link1, const KinBody::Link& link2) const {
  return m_pairs.find(LinkPair(&link1, &link2)) == m_pairs.end();
}


CollisionCheckerPtr CreateCollisionChecker(OR::EnvironmentBaseConstPtr env) {
  CollisionCheckerPtr checker(new BulletCollisionChecker(env));
  return checker;
}
}
