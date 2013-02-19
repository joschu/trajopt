#include "trajopt/collision_checker.hpp"
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h>
#include <openrave-core.h>
#include "utils/eigen_conversions.hpp"
#include <boost/foreach.hpp>
#include <vector>
#include <iostream>
#include <LinearMath/btConvexHull.h>
#include <utils/stl_to_string.hpp>
using namespace util;
using namespace std;
using namespace trajopt;
using namespace OpenRAVE;

namespace {

enum CollisionFilterGroups {
  RobotFilter = 1,
  KinBodyFilter = 2
};

const float MARGIN = 0;

ostream &operator<<(ostream &stream, const btVector3& v) {
  stream << v.x() << " " << v.y() << " " << v.z();
  return stream;
}

class CollisionObjectWrapper : public btCollisionObject {
public:
  CollisionObjectWrapper(KinBody::Link* link) : m_link(link), m_index(-1) {}
  vector<boost::shared_ptr<void> > m_data;
  KinBody::Link* m_link;
  int m_index; // index into collision matrix
  template<class T>
  void manage(T* t) { // manage memory of this object
    m_data.push_back(boost::shared_ptr<T>(t));
  }
  template<class T>
  void manage(shared_ptr<T> t) {
    m_data.push_back(t);
  }
};
typedef CollisionObjectWrapper COW;
typedef boost::shared_ptr<CollisionObjectWrapper> COWPtr;

inline const KinBody::Link* getLink(const btCollisionObject* o) {
  return static_cast<const CollisionObjectWrapper*>(o)->m_link;
}


extern void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo);




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

bool isIdentity(const OpenRAVE::Transform& T) {
  float e = 1e-7;
  return
      fabs(T.trans.x) < e &&
      fabs(T.trans.y) < e &&
      fabs(T.trans.z) < e &&
      fabs(T.rot[0]-1) < e &&
      fabs(T.rot[1]) < e &&
      fabs(T.rot[2]) < e &&
      fabs(T.rot[3]) < e;
}




btCollisionShape* createShapePrimitive(OR::KinBody::Link::GeometryPtr geom, bool useTrimesh, CollisionObjectWrapper* cow) {

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
    float r = geom->GetCylinderRadius(), h = geom->GetCylinderHeight() / 2;
    subshape = new btCylinderShapeZ(btVector3(r, r, h / 2));
    break;
  }
  case KinBody::Link::GEOMPROPERTIES::GeomTrimesh: {
    const KinBody::Link::TRIMESH &mesh = geom->GetCollisionMesh();
    assert(mesh.indices.size() >= 3);
    boost::shared_ptr<btTriangleMesh> ptrimesh(new btTriangleMesh());

    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
      ptrimesh->addTriangle(toBt(mesh.vertices[mesh.indices[i]]), toBt(mesh.vertices[mesh.indices[i + 1]]),
              toBt(mesh.vertices[mesh.indices[i + 2]]));
    }

    if (useTrimesh) {
      subshape = new btBvhTriangleMeshShape(ptrimesh.get(), true);
      cow->manage(ptrimesh);
    }
    else { // CONVEX HULL
      btConvexTriangleMeshShape convexTrimesh(ptrimesh.get());
      convexTrimesh.setMargin(MARGIN); // margin: hull padding
      //Create a hull shape to approximate Trimesh

      bool useShapeHull;

      btShapeHull shapeHull(&convexTrimesh);
      if (mesh.vertices.size() >= 50) {
        bool success = shapeHull.buildHull(-666); // note: margin argument not used
        if (!success) RAVELOG_WARN("shapehull convex hull failed! falling back to original vertices\n");
        useShapeHull = success;
      }
      else {
        useShapeHull = false;
      }

      btConvexHullShape *convexShape = new btConvexHullShape();
      subshape = convexShape;
      if (useShapeHull) {
        for (int i = 0; i < shapeHull.numVertices(); ++i)
          convexShape->addPoint(shapeHull.getVertexPointer()[i]);
        break;
      }
      else {
        for (int i = 0; i < mesh.vertices.size(); ++i)
          convexShape->addPoint(toBt(mesh.vertices[i]));
        break;
      }
      
    }     
  }
  default:
    assert(0 && "unrecognized collision shape type");
    break;
  }
  return subshape;
}


COWPtr CollisionObjectFromLink(OR::KinBody::LinkPtr link, bool useTrimesh) {
  RAVELOG_DEBUG("creating bt collision object from from %s\n",link->GetName().c_str());

  const std::vector<boost::shared_ptr<OpenRAVE::KinBody::Link::GEOMPROPERTIES> > & geometries=link->GetGeometries();

  if (geometries.empty()) return COWPtr();

  COWPtr cow(new CollisionObjectWrapper(link.get()));

  if ( false && (link->GetGeometries().size() == 1) && isIdentity(link->GetGeometry(0)->GetTransform())) {
    btCollisionShape* shape = createShapePrimitive(link->GetGeometry(0), useTrimesh, cow.get());
    shape->setMargin(MARGIN);
    cow->manage(shape);
    cow->setCollisionShape(shape);

  }
  else {
    btCompoundShape* compound = new btCompoundShape(/*dynamicAABBtree=*/false);
    cow->manage(compound);
    compound->setMargin(MARGIN); //margin: compound. seems to have no effect when positive but has an effect when negative
    cow->setCollisionShape(compound);

    BOOST_FOREACH(const boost::shared_ptr<OpenRAVE::KinBody::Link::GEOMPROPERTIES>& geom, geometries) {

      btCollisionShape* subshape = createShapePrimitive(geom, useTrimesh, cow.get());
      if (subshape != NULL) {
        cow->manage(subshape);
        subshape->setMargin(MARGIN);
        btTransform geomTrans = toBt(geom->GetTransform());
        compound->addChildShape(geomTrans, subshape);
      }
    }

  }

  cow->setWorldTransform(toBt(link->GetTransform()));

  return cow;
}



void RenderCollisionShape(btCollisionShape* shape, const btTransform& tf,
    OpenRAVE::EnvironmentBase& env, vector<OpenRAVE::GraphHandlePtr>& handles) {

  typedef map<btCollisionShape*, HullResult > Shape2Inds;
  Shape2Inds gHullCache;

  switch (shape->getShapeType()) {
  case COMPOUND_SHAPE_PROXYTYPE: {
    btCompoundShape* compound = static_cast<btCompoundShape*>(shape);
    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
      RenderCollisionShape(compound->getChildShape(i),
          tf * compound->getChildTransform(i), env, handles);
    }
    break;
  }
  case CONVEX_HULL_SHAPE_PROXYTYPE: {
    btConvexHullShape* convex = static_cast<btConvexHullShape*>(shape);

    Shape2Inds::iterator it = gHullCache.find(convex);

    btAlignedObjectArray<unsigned int> inds;
    HullResult hr;
    if ( it != gHullCache.end() )
      hr = it->second;
    else {

      HullDesc hd;
      hd.mFlags = QF_TRIANGLES;
      hd.mVcount = convex->getNumPoints();
      hd.mVertices = convex->getPoints();
      hd.mVertexStride = sizeof(btVector3);
      HullLibrary hl;

      if (hl.CreateConvexHull(hd, hr) == QE_FAIL) {
        RAVELOG_ERROR("convex hull computation failed on shape with %i vertices\n", convex->getNumPoints());
        hr.mNumFaces = 0;
      }
      else {
      }
      gHullCache[convex] = hr;
    }

    if (hr.mNumFaces > 0) {
      vector<btVector3> tverts(hr.mNumOutputVertices);
      for (int i=0; i < tverts.size(); ++i) tverts[i] = tf * hr.m_OutputVertices[i];


      handles.push_back(env.drawtrimesh((float*)&tverts[0], 16,
          (int*) &hr.m_Indices[0], hr.mNumFaces, OR::RaveVector<float>(1,1,1,.1)));
    }
    break;


  }

  default:
    RAVELOG_INFO("not rendering shape of type %i\n", shape->getShapeType());
    break;
  }
}


class BulletCollisionChecker : public CollisionChecker {
  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;
  typedef map<const OR::KinBody::Link*, CollisionObjectWrapper*> Link2Cow;
  Link2Cow m_link2cow;
  double m_contactDistance;
  vector<KinBodyPtr> m_prevbodies;
  typedef std::pair<const KinBody::Link*, const KinBody::Link*> LinkPair;
  set< LinkPair > m_excludedPairs;
  Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> m_allowedCollisionMatrix;

public:
  BulletCollisionChecker(OR::EnvironmentBaseConstPtr env);
  ~BulletCollisionChecker();

  ///////// public interface /////////
  virtual void SetContactDistance(float distance);
  virtual double GetContactDistance() {return m_contactDistance;}
  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>& handles);
  virtual void ExcludeCollisionPair(const KinBody::Link& link0, const KinBody::Link& link1) {
    m_excludedPairs.insert(LinkPair(&link0, &link1));
    m_allowedCollisionMatrix(GetCow(&link0)->m_index, GetCow(&link1)->m_index) = 0;
  }
  // collision checking
  virtual void AllVsAll(vector<Collision>& collisions);
  virtual void LinksVsAll(const vector<KinBody::LinkPtr>& links, vector<Collision>& collisions);
  virtual void LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions);
  virtual void ContinuousCheckTrajectory(const TrajArray& traj, RobotAndDOF& rad, vector<Collision>&);
  virtual void CastVsAll(RobotAndDOF& rad, const vector<KinBody::LinkPtr>& links, const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions);
  ////
  ///////

  CollisionObjectWrapper* GetCow(const KinBody::Link* link) {
    Link2Cow::iterator it = m_link2cow.find(link);
    return (it == m_link2cow.end()) ? 0 : it->second;
  }
  void SetCow(const KinBody::Link* link, COW* cow) {m_link2cow[link] = cow;}
  void LinkVsAll_NoUpdate(const KinBody::Link& link, vector<Collision>& collisions);
  void UpdateBulletFromRave();
  void AddKinBody(const OR::KinBodyPtr& body);
  void RemoveKinBody(const OR::KinBodyPtr& body);
  void AddAndRemoveBodies(const vector<OR::KinBodyPtr>& curVec, const vector<OR::KinBodyPtr>& prevVec, vector<KinBodyPtr>& addedBodies);
  bool CanCollide(const CollisionObjectWrapper* cow0, const CollisionObjectWrapper* cow1) {
    return m_allowedCollisionMatrix(cow0->m_index, cow1->m_index);
  }
  void SetLinkIndices();
  void CheckShapeCast(btCollisionShape* shape, const btTransform& tf0, const btTransform& tf1,
      CollisionObjectWrapper* cow, btCollisionWorld* world, vector<Collision>& collisions);


};

struct CollisionCollector : public btCollisionWorld::ContactResultCallback {
  std::vector<Collision>& m_collisions;
  const CollisionObjectWrapper* m_cow;
  BulletCollisionChecker* m_cc;

  CollisionCollector(vector<Collision>& collisions, CollisionObjectWrapper* cow, BulletCollisionChecker* cc) :
    m_collisions(collisions), m_cow(cow), m_cc(cc) {}
  virtual btScalar addSingleResult(btManifoldPoint& cp,
      const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
      const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1) {
    if (cp.m_distance1 > m_cc->GetContactDistance()) return 0;
    const KinBody::Link* linkA = getLink(colObj0Wrap->getCollisionObject());
    const KinBody::Link* linkB = getLink(colObj1Wrap->getCollisionObject());
    m_collisions.push_back(Collision(linkA, linkB, toOR(cp.m_positionWorldOnA), toOR(cp.m_positionWorldOnB),
        toOR(cp.m_normalWorldOnB), cp.m_distance1));
    RAVELOG_DEBUG("collide %s-%s\n", linkA->GetName().c_str(), linkB->GetName().c_str());
    return 0;
  }
  bool needsCollision(btBroadphaseProxy* proxy0) const {
    bool maskcollides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
    maskcollides = maskcollides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
    return maskcollides && m_cc->CanCollide(m_cow, static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject));
  }
};


// only used for AllVsAll
void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {
  BulletCollisionChecker* cc = static_cast<BulletCollisionChecker*>(dispatcher.m_userData);
  if ( cc->CanCollide(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy0->m_clientObject),
                      static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy1->m_clientObject)))
    dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);
}


BulletCollisionChecker::BulletCollisionChecker(OR::EnvironmentBaseConstPtr env) :
  CollisionChecker(env) {
  m_coll_config = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_coll_config);
  m_broadphase = new btDbvtBroadphase();
  m_world = new btCollisionWorld(m_dispatcher, m_broadphase, m_coll_config);
  m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,
      m_coll_config->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));
  m_dispatcher->setNearCallback(&nearCallback);
  m_dispatcher->m_userData = this;
  SetContactDistance(.05);
  UpdateBulletFromRave();
}

BulletCollisionChecker::~BulletCollisionChecker() {
  delete m_world;
  delete m_broadphase;
  delete m_dispatcher;
  delete m_coll_config;
}


void BulletCollisionChecker::SetContactDistance(float dist) {
  RAVELOG_DEBUG("setting contact distance to %.2f\n", dist);
  m_contactDistance = dist;
  SHAPE_EXPANSION = btVector3(1,1,1)*dist;
  gContactBreakingThreshold = 2.001*dist; // wtf. when I set it to 2.0 there are no contacts with distance > 0
  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  for (int i=0; i < objs.size(); ++i) {
    objs[i]->setContactProcessingThreshold(dist);
  }
  btCollisionDispatcher* dispatcher = static_cast<btCollisionDispatcher*>(m_world->getDispatcher());
  dispatcher->setDispatcherFlags(dispatcher->getDispatcherFlags() & ~btCollisionDispatcher::CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD);
}


void BulletCollisionChecker::AllVsAll(vector<Collision>& collisions) {
  UpdateBulletFromRave();
  RAVELOG_DEBUG("AllVsAll\n");
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
//      stringstream ss; ss << pt.m_localPointA << " | " << pt.m_localPointB;
//      RAVELOG_DEBUG("local pts: %s\n",ss.str().c_str());
      // adjustContactPoint(pt, objA, objB);

      const KinBody::Link* bodyA = objA->m_link;
      const KinBody::Link* bodyB = objB->m_link;

      if (CanCollide(objA, objB)) {
        collisions.push_back(Collision(bodyA, bodyB, toOR(pt.getPositionWorldOnA()), toOR(pt.getPositionWorldOnB()),
            toOR(pt.m_normalWorldOnB), pt.m_distance1, 1./numContacts));
      }
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

void BulletCollisionChecker::LinksVsAll(const vector<KinBody::LinkPtr>& links, vector<Collision>& collisions) {
//  AllVsAll(collisions);
//  return;

  UpdateBulletFromRave();
  m_world->updateAabbs();

  BOOST_FOREACH(const KinBody::LinkPtr& link, links) {
    LinkVsAll_NoUpdate(*link, collisions);// xxx just testing
  }
}


void BulletCollisionChecker::LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions) {
  UpdateBulletFromRave();
  LinkVsAll_NoUpdate(link, collisions);
}

void BulletCollisionChecker::LinkVsAll_NoUpdate(const KinBody::Link& link, vector<Collision>& collisions) {
  if (link.GetGeometries().empty()) return;
  CollisionObjectWrapper* cow = GetCow(&link);
  CollisionCollector cc(collisions, cow, this);
  m_world->contactTest(cow, cc);
}

class KinBodyCollisionData;
typedef boost::shared_ptr<KinBodyCollisionData> CDPtr;
struct KinBodyCollisionData : public OpenRAVE::UserData {
  OpenRAVE::KinBodyWeakPtr body;
  std::vector<KinBody::Link*> links;
  std::vector<COWPtr> cows;
  KinBodyCollisionData(OR::KinBodyPtr _body) : body(_body) {}
};

void BulletCollisionChecker::AddKinBody(const OR::KinBodyPtr& body) {
  CDPtr cd(new KinBodyCollisionData(body));

  int filterGroup = body->IsRobot() ? RobotFilter : KinBodyFilter;
  const vector<OR::KinBody::LinkPtr> links = body->GetLinks();

  body->SetUserData("bt", cd);
  
  bool useTrimesh = body->GetUserData("bt_use_trimesh");
  BOOST_FOREACH(const OR::KinBody::LinkPtr& link, links) {
    if (link->GetGeometries().size() > 0) {
      COWPtr new_cow = CollisionObjectFromLink(link, useTrimesh); 
      if (new_cow) {
        SetCow(link.get(), new_cow.get());
        m_world->addCollisionObject(new_cow.get(), filterGroup);
        new_cow->setContactProcessingThreshold(m_contactDistance);
        RAVELOG_DEBUG("added collision object for  link %s\n", link->GetName().c_str());
        cd->links.push_back(link.get());
        cd->cows.push_back(new_cow);
      }
      else {
        RAVELOG_WARN("ignoring link %s\n", link->GetName().c_str());
      }
    }
  }

}
void BulletCollisionChecker::RemoveKinBody(const OR::KinBodyPtr& body) {
  RAVELOG_DEBUG("removing %s\n", body->GetName().c_str());
  BOOST_FOREACH(const OR::KinBody::LinkPtr& link, body->GetLinks()) {
    CollisionObjectWrapper* cow = GetCow(link.get());
    if (cow) {
      m_world->removeCollisionObject(cow);
      m_link2cow.erase(link.get());      
    }
  }
  body->RemoveUserData("bt");
}

template <typename T>
void SetDifferences(const vector<T>& A, const vector<T>& B, vector<T>& AMinusB, vector<T>& BMinusA) {
  set<T> Aset, Bset;
  AMinusB.clear();
  BMinusA.clear();
  BOOST_FOREACH(const T& a, A) {
    Aset.insert(a);
  }
  BOOST_FOREACH(const T& b, B) {
    Bset.insert(b);
  }
  BOOST_FOREACH(const T& a, A) {
    if (Bset.count(a) == 0) AMinusB.push_back(a);
  }
  BOOST_FOREACH(const T& b, B) {
    if (Aset.count(b) == 0) BMinusA.push_back(b);
  }
}

void BulletCollisionChecker::AddAndRemoveBodies(const vector<KinBodyPtr>& curVec, const vector<KinBodyPtr>& prevVec, vector<KinBodyPtr>& toAdd) {
  vector<KinBodyPtr> toRemove;
  SetDifferences(curVec, prevVec, toAdd, toRemove);
  BOOST_FOREACH(const KinBodyPtr& body, toAdd) {
    assert(!body->GetUserData("bt"));
    AddKinBody(body);
  }
  BOOST_FOREACH(const KinBodyPtr& body, toRemove) {
    RemoveKinBody(body);
  }
  SetLinkIndices();
}

void BulletCollisionChecker::SetLinkIndices() {
  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  for (int i=0; i < objs.size(); ++i) {
    CollisionObjectWrapper* cow = static_cast<CollisionObjectWrapper*>(objs[i]);
    cow->m_index = i;
  }
  m_allowedCollisionMatrix.resize(objs.size(), objs.size());
  m_allowedCollisionMatrix.setOnes();
  BOOST_FOREACH(const LinkPair& pair, m_excludedPairs) {
    const KinBody::Link* linkA = pair.first;
    const KinBody::Link* linkB = pair.second;
    const CollisionObjectWrapper* cowA = GetCow(linkA);
    const CollisionObjectWrapper* cowB = GetCow(linkA);
    assert(cowA != NULL && cowB != NULL);
    m_allowedCollisionMatrix(cowA->m_index, cowB->m_index) = 0;
    m_allowedCollisionMatrix(cowB->m_index, cowA->m_index) = 0;
  }

}

void BulletCollisionChecker::UpdateBulletFromRave() {
  vector<OR::KinBodyPtr> bodies, addedBodies;
  m_env->GetBodies(bodies);
  if (bodies.size() != m_prevbodies.size() || !std::equal(bodies.begin(), bodies.end(), m_prevbodies.begin())) {
    RAVELOG_DEBUG("need to add and remove stuff\n");
    AddAndRemoveBodies(bodies, m_prevbodies, addedBodies);
    m_prevbodies=bodies;
    BOOST_FOREACH(const KinBodyPtr& body, addedBodies) {
      IgnoreZeroStateSelfCollisions(body);
    }
  }
  else {
    RAVELOG_DEBUG("don't need to add or remove stuff\n");
  }

  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  RAVELOG_DEBUG("%i objects in bullet world\n", objs.size());
  for (int i=0; i < objs.size(); ++i) {
    CollisionObjectWrapper* cow = static_cast<CollisionObjectWrapper*>(objs[i]);
    cow->setWorldTransform(toBt(cow->m_link->GetTransform()));
  }

}


void BulletCollisionChecker::PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>& handles) {
  UpdateBulletFromRave();
  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  RAVELOG_DEBUG("%i objects in bullet world\n", objs.size());
  for (int i=0; i < objs.size(); ++i) {
    RenderCollisionShape(objs[i]->getCollisionShape(), objs[i]->getWorldTransform(), *boost::const_pointer_cast<OpenRAVE::EnvironmentBase>(m_env), handles);
  }
}




////////// Continuous collisions ////////////////////////

namespace {

vector<btTransform> rightMultiplyAll(const vector<btTransform>& xs, const btTransform& y) {
  vector<btTransform> out(xs.size());
  for (int i=0; i < xs.size(); ++i) out[i] = xs[i]*y;
  return out;
}


}

void ContinuousCheckShape(btCollisionShape* shape, const vector<btTransform>& transforms,
    KinBody::Link* link, btCollisionWorld* world, vector<Collision>& collisions) {
  if (btConvexShape* convex = dynamic_cast<btConvexShape*>(shape)) {
    for (int i=0; i < transforms.size()-1; ++i) {
      btCollisionWorld::ClosestConvexResultCallback ccc(btVector3(NAN, NAN, NAN), btVector3(NAN, NAN, NAN));
      ccc.m_collisionFilterMask = KinBodyFilter;
      world->convexSweepTest(convex, transforms[i], transforms[i+1], ccc, 0);
      if (ccc.hasHit()) {
        collisions.push_back(Collision(link, getLink(ccc.m_hitCollisionObject),
            toOR(ccc.m_hitPointWorld), toOR(ccc.m_hitPointWorld), toOR(ccc.m_hitNormalWorld), 0, 1, i+ccc.m_closestHitFraction));
      }
    }
  }
  else if (btCompoundShape* compound = dynamic_cast<btCompoundShape*>(shape)) {
    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
      ContinuousCheckShape(compound->getChildShape(i), rightMultiplyAll(transforms, compound->getChildTransform(i)),  link, world, collisions);
    }
  }
  else {
    throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
  }

}


void BulletCollisionChecker::ContinuousCheckTrajectory(const TrajArray& traj, RobotAndDOF& rad, vector<Collision>& collisions) {
  UpdateBulletFromRave();
  m_world->updateAabbs();

  // first calculate transforms of all the relevant links at each step
  vector<KinBody::LinkPtr> links;
  vector<int> link_inds;
  rad.GetAffectedLinks(links, true, link_inds);


  // don't need to remove them anymore because now I only check collisions
  // against KinBodyFilter stuff
  // remove them, because we can't check moving stuff against each other
  vector<CollisionObjectWrapper*> cows;
  BOOST_FOREACH(KinBody::LinkPtr& link, links) {
    CollisionObjectWrapper* cow = GetCow(link.get());
    assert(cow != NULL);
    cows.push_back(cow);
#if 0
    m_world->removeCollisionObject(cow);
#endif
  }


  typedef vector<btTransform> TransformVec;
  vector<TransformVec> link2transforms(links.size(), TransformVec(traj.rows()));
  RobotBase::RobotStateSaver save = rad.Save();

  for (int iStep=0; iStep < traj.rows(); ++iStep) {
    rad.SetDOFValues(toDblVec(traj.row(iStep)));
    for (int iLink = 0; iLink < links.size(); ++iLink) {
      link2transforms[iLink][iStep] = toBt(links[iLink]->GetTransform());
    }
  }

  for (int iLink = 0; iLink < links.size(); ++iLink) {
    ContinuousCheckShape(cows[iLink]->getCollisionShape(), link2transforms[iLink], links[iLink].get(), m_world, collisions);
  }

#if 0
  // add them back
  BOOST_FOREACH(CollisionObjectWrapper* cow, cows) {
    m_world->addCollisionObject(cow);
  }
#endif
}

#if 0
class CompoundHullShape : public btConvexShape {
  std::vector<btConvexHullShape*> m_children;
  btVector3   localGetSupportingVertex(const btVector3& vec)const {
    btVector3 sv = m_children[0]->localGetSupportingVertex(vec);
    float support = sv.dot(vec);
    for (int i=1; i < m_children.size(); ++i) {
      btVector3 newsv = m_children[i]->localGetSupportingVertex(vec);
      float newsupport = vec.dot(newsv);
      if (newsupport > support) {
        support = newsupport;
        sv = newsv;
      }
    }
  }
#if 0
  void project(const btTransform& trans, const btVector3& dir, btScalar& min, btScalar& max) const {
    m_children[0]->project(trans, dir, min, max);
    for (int i=1; i < m_children.size(); ++i) {
      btScalar newmin, newmax;
      m_children[i]->project(trans, dir, newmin, newmax);
      btSetMin(min, newmin);
      btSetMax(max, newmax);
    }
  }
#endif

  //notice that the vectors should be unit length
  void    batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const {
    throw std::runtime_error("not implemented");
  }

  ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const {
    m_children[0]->getAabb(t, aabbMin, aabbMax);
    for (int i=1; i < m_children.size(); ++i) {
      btVector3 newmin, newmax;
      m_children[i]->getAabb(t, newmin, newmax);
      aabbMin.setMin(newmin);
      aabbMax.setMax(newmax);
    }
  }

  virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const {
    throw std::runtime_error("shouldn't happen");
  }

  virtual void    setLocalScaling(const btVector3& scaling) {}
  virtual const btVector3& getLocalScaling() const {return btVector3(1,1,1);}

  virtual void    setMargin(btScalar margin) {}
  virtual btScalar    getMargin() const {return 0;}

  virtual int     getNumPreferredPenetrationDirections() const {return 0;}
  virtual void    getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const=0;


};
#endif


class CastHullShape : public btConvexShape {
public:
  btConvexShape* m_shape;
  btTransform m_t01, m_t10; // T_0_1 = T_w_0^-1 * T_w_1
  CastHullShape(btConvexShape* shape, const btTransform& t01) : m_shape(shape), m_t01(t01) {
    m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE;



  }
  btVector3   localGetSupportingVertex(const btVector3& vec)const {
    btVector3 sv0 = m_shape->localGetSupportingVertex(vec);
    btVector3 sv1 = m_t01*m_shape->localGetSupportingVertex(vec*m_t01.getBasis());
    return (vec.dot(sv0) > vec.dot(sv1)) ? sv0 : sv1;
  }
#if 0
  void project(const btTransform& trans, const btVector3& dir, btScalar& min, btScalar& max) const {
    m_children[0]->project(trans, dir, min, max);
    for (int i=1; i < m_children.size(); ++i) {
      btScalar newmin, newmax;
      m_children[i]->project(trans, dir, newmin, newmax);
      btSetMin(min, newmin);
      btSetMax(max, newmax);
    }
  }
#endif

  //notice that the vectors should be unit length
  void    batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const {
    throw std::runtime_error("not implemented");
  }

  ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
  void getAabb(const btTransform& t_w0,btVector3& aabbMin,btVector3& aabbMax) const {
    m_shape->getAabb(t_w0, aabbMin, aabbMax);
    btVector3 min1, max1;
    m_shape->getAabb(t_w0*m_t01, min1, max1 );
    aabbMin.setMin(min1);
    aabbMax.setMax(max1);
  }

  virtual void getAabbSlow(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const {
    throw std::runtime_error("shouldn't happen");
  }

  virtual void    setLocalScaling(const btVector3& scaling) {}
  virtual const btVector3& getLocalScaling() const {
    static btVector3 out(1,1,1);
    return out;
  }

  virtual void    setMargin(btScalar margin) {}
  virtual btScalar    getMargin() const {return 0;}

  virtual int     getNumPreferredPenetrationDirections() const {return 0;}
  virtual void    getPreferredPenetrationDirection(int index, btVector3& penetrationVector) const {throw std::runtime_error("not implemented");}


  virtual void calculateLocalInertia(btScalar, btVector3&) const {throw std::runtime_error("not implemented");}
  virtual const char* getName() const {return "CastHull";}
  virtual btVector3 localGetSupportingVertexWithoutMargin(const btVector3& v) const {return localGetSupportingVertex(v);}


};



void BulletCollisionChecker::CheckShapeCast(btCollisionShape* shape, const btTransform& tf0, const btTransform& tf1,
    CollisionObjectWrapper* cow, btCollisionWorld* world, vector<Collision>& collisions) {
  if (btConvexShape* convex = dynamic_cast<btConvexShape*>(shape)) {
    CastHullShape* shape = new CastHullShape(convex, tf0.inverseTimes(tf1));
    CollisionObjectWrapper* obj = new CollisionObjectWrapper(cow->m_link);
    obj->setCollisionShape(shape);
    obj->setWorldTransform(tf0);
    obj->m_index = cow->m_index;
    CollisionCollector cc(collisions, obj, this);
    cc.m_collisionFilterMask = KinBodyFilter;
    cc.m_collisionFilterGroup = RobotFilter;
    world->contactTest(obj, cc);
    delete obj;
    delete shape;
  }
  else if (btCompoundShape* compound = dynamic_cast<btCompoundShape*>(shape)) {
    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
      CheckShapeCast(compound->getChildShape(i), tf0*compound->getChildTransform(i), tf1*compound->getChildTransform(i), cow, world, collisions);
    }
  }
  else {
    throw std::runtime_error("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
  }

}

void BulletCollisionChecker::CastVsAll(RobotAndDOF& rad, const vector<KinBody::LinkPtr>& links,
    const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions) {
  OR::RobotBase::RobotStateSaver saver = rad.Save();
  rad.SetDOFValues(startjoints);
  int nlinks = links.size();
  vector<btTransform> tbefore(nlinks), tafter(nlinks);
  for (int i=0; i < nlinks; ++i) {
    tbefore[i] = toBt(links[i]->GetTransform());
  }
  rad.SetDOFValues(endjoints);
  for (int i=0; i < nlinks; ++i) {
    tafter[i] = toBt(links[i]->GetTransform());
  }
  rad.SetDOFValues(startjoints);
  UpdateBulletFromRave();
  m_world->updateAabbs();

  for (int i=0; i < nlinks; ++i) {
    assert(m_link2cow[links[i].get()] != NULL);
    CollisionObjectWrapper* cow = m_link2cow[links[i].get()];
    CheckShapeCast(cow->getCollisionShape(), tbefore[i], tafter[i], cow, m_world, collisions);
  }
  RAVELOG_DEBUG("CastVsAll checked %i links and found %i collisions\n", links.size(), collisions.size());
}

}






namespace trajopt {



CollisionCheckerPtr CreateCollisionChecker(OR::EnvironmentBaseConstPtr env) {
  CollisionCheckerPtr checker(new BulletCollisionChecker(env));
  return checker;
}
}
