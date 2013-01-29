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
  CollisionObjectWrapper(KinBody::Link* link) : m_link(link) {}
  vector<boost::shared_ptr<void> > m_data;
  KinBody::Link* m_link;
  template<class T>
  void manage(T* t) { // manage memory of this object
    m_data.push_back(boost::shared_ptr<T>(t));
  }
};
typedef CollisionObjectWrapper COW;
typedef boost::shared_ptr<CollisionObjectWrapper> COWPtr;

inline const KinBody::Link* getLink(const btCollisionObject* o) {
  return static_cast<const CollisionObjectWrapper*>(o)->m_link;
}


void nearCallback(btBroadphasePair& collisionPair,
    btCollisionDispatcher& dispatcher, const btDispatcherInfo& dispatchInfo) {
  CollisionPairIgnorer* ignorer = static_cast<CollisionPairIgnorer*>(dispatcher.m_userData);
  KinBody::Link* linkA = static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy0->m_clientObject)->m_link;
  KinBody::Link* linkB = static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy1->m_clientObject)->m_link;
  if ( ignorer->CanCollide(*linkA, *linkB))
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




btCollisionShape* createShapePrimitive(OR::KinBody::Link::GeometryPtr geom) {

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
    btTriangleMesh ptrimesh;

    for (size_t i = 0; i < mesh.indices.size(); i += 3) {
      ptrimesh.addTriangle(toBt(mesh.vertices[mesh.indices[i]]), toBt(mesh.vertices[mesh.indices[i + 1]]),
              toBt(mesh.vertices[mesh.indices[i + 2]]));
    }

    btConvexTriangleMeshShape convexTrimesh(&ptrimesh);
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
  default:
    assert(0 && "unrecognized collision shape type");
    break;
  }
  return subshape;
}

COWPtr CollisionObjectFromLink(OR::KinBody::LinkPtr link) {
  RAVELOG_DEBUG("creating bt collision object from from %s\n",link->GetName().c_str());

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

  if ( false && (link->GetGeometries().size() == 1) && isIdentity(link->GetGeometry(0)->GetTransform())) {
    btCollisionShape* shape = createShapePrimitive(link->GetGeometry(0));
    shape->setMargin(MARGIN);
    cow->manage(shape);
    cow->setCollisionShape(shape);

  }
  else {
    btCompoundShape* compound = new btCompoundShape(false);
    cow->manage(compound);
    compound->setMargin(MARGIN); //margin: compound. seems to have no effect when positive but has an effect when negative
    cow->setCollisionShape(compound);

    BOOST_FOREACH(const boost::shared_ptr<OpenRAVE::KinBody::Link::GEOMPROPERTIES>& geom, geometries) {

      btCollisionShape* subshape = createShapePrimitive(geom);
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


typedef map<btCollisionShape*, HullResult > Shape2Inds;
Shape2Inds gHullCache;

void RenderCollisionShape(btCollisionShape* shape, const btTransform& tf,
    OpenRAVE::EnvironmentBase& env, vector<OpenRAVE::GraphHandlePtr>& handles) {
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

struct CollisionCollector : public btCollisionWorld::ContactResultCallback {
  std::vector<Collision>& m_collisions;
  const CollisionPairIgnorer& m_ignorer;
  const KinBody::Link& m_link;
  CollisionCollector(vector<Collision>& collisions, const CollisionPairIgnorer& ignorer, const KinBody::Link& link) :
    m_collisions(collisions), m_ignorer(ignorer), m_link(link) {}
  virtual btScalar addSingleResult(btManifoldPoint& cp,
      const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
      const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1) {
    const KinBody::Link* linkA = getLink(colObj0Wrap->getCollisionObject());
    const KinBody::Link* linkB = getLink(colObj1Wrap->getCollisionObject());
//    if ( m_ignorer.CanCollide(*linkA, *linkB)) { we're checking in needscollision
    if (true) {
      m_collisions.push_back(Collision(linkA, linkB, toOR(cp.m_positionWorldOnA), toOR(cp.m_positionWorldOnB),
          toOR(cp.m_normalWorldOnB), cp.m_distance1));
      RAVELOG_DEBUG("collide %s-%s\n", linkA->GetName().c_str(), linkB->GetName().c_str());
    }

    return 0;
  }
  bool needsCollision(btBroadphaseProxy* proxy0) const {
    bool maskcollides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
    maskcollides = maskcollides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);
    KinBody::Link* otherlink = static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)->m_link;
    if (maskcollides) {
      KinBody::Link* otherlink = static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)->m_link;
      bool out = m_ignorer.CanCollide(m_link, *otherlink);
      return out;
    }
    else return false;
  }
};

class BulletCollisionChecker : public CollisionChecker {
  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;
  typedef map<const OR::KinBody::Link*, CollisionObjectWrapper*> Link2Cow;
  Link2Cow m_link2cow;
  double m_contactDistance;

public:
  BulletCollisionChecker(OR::EnvironmentBaseConstPtr env);
  ~BulletCollisionChecker();
  void SetContactDistance(float distance);
  double GetContactDistance() {return m_contactDistance;}
  void AllVsAll(vector<Collision>& collisions);
  virtual void LinksVsAll(const vector<KinBody::LinkPtr>& links, vector<Collision>& collisions);
  virtual void LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions);
  virtual void UpdateBulletFromRave();
  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>& handles);
  virtual void ContinuousCheckTrajectory(const TrajArray& traj, RobotAndDOF& rad, vector<Collision>&);
  virtual void CastVsAll(RobotAndDOF& rad, const vector<KinBody::LinkPtr>& links, const DblVec& startjoints, const DblVec& endjoints, vector<Collision>& collisions);

  CollisionObjectWrapper* GetCow(const KinBody::Link* link) {
    Link2Cow::iterator it = m_link2cow.find(link);
    return (it == m_link2cow.end()) ? 0 : it->second;
  }
  void SetCow(const KinBody::Link* link, COW* cow) {m_link2cow[link] = cow;}

  void AddKinBody(const OR::KinBodyPtr& body);
  void RemoveKinBody(const OR::KinBodyPtr& body);

};



BulletCollisionChecker::BulletCollisionChecker(OR::EnvironmentBaseConstPtr env) :
  CollisionChecker(env) {
  m_coll_config = new btDefaultCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_coll_config);
  m_broadphase = new btDbvtBroadphase();
  m_world = new btCollisionWorld(m_dispatcher, m_broadphase, m_coll_config);
  m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,
      m_coll_config->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));
  m_dispatcher->setNearCallback(&nearCallback);
  m_dispatcher->m_userData = &m_ignorer;
  SetContactDistance(.05);
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

      if (m_ignorer.CanCollide(*bodyA, *bodyB)) {
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
    LinkVsAll(*link, collisions);// xxx just testing
  }
}


void BulletCollisionChecker::LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions) {
  if (link.GetGeometries().empty()) return;
  CollisionObjectWrapper* cow = GetCow(&link);
  CollisionCollector cc(collisions, m_ignorer, link);
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
  BOOST_FOREACH(const OR::KinBody::LinkPtr& link, links) {
    if (link->GetGeometries().size() > 0) {
      COWPtr new_cow = CollisionObjectFromLink(link);
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

//  const std::set<int>& pairhashes = body->GetAdjacentLinks();
//  BOOST_FOREACH(const int& p, pairhashes) {
//    RAVELOG_DEBUG("excluding pair %s %s\n", links[p & 0xffff]->GetName().c_str(), links[p >> 16]->GetName().c_str());
//    m_ignorer.ExcludePair(*links[p & 0xffff], *links[p >> 16]);
//  }
}
void BulletCollisionChecker::RemoveKinBody(const OR::KinBodyPtr& body) {
  BOOST_FOREACH(const OR::KinBody::LinkPtr& link, body->GetLinks()) {
    CollisionObjectWrapper* cow = GetCow(link.get());
    m_world->removeCollisionObject(cow);
    m_link2cow.erase(link.get());
    body->RemoveUserData("bt");
  }
}


void BulletCollisionChecker::UpdateBulletFromRave() {


  vector<OR::KinBodyPtr> bodies;
  m_env->GetBodies(bodies);
  vector<OR::KinBodyPtr> bodies_added;
  BOOST_FOREACH(const OR::KinBodyPtr& body, bodies) {
    if (!body->GetUserData("bt")) {
      AddKinBody(body);
      bodies_added.push_back(body);
    }
  }
  BOOST_FOREACH(const OR::KinBodyPtr& body, bodies_added) {
    IgnoreZeroStateSelfCollisions(body);
  }


  btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
  RAVELOG_DEBUG("%i objects in bullet world\n", objs.size());
  for (int i=0; i < objs.size(); ++i) {
    CollisionObjectWrapper* cow = static_cast<CollisionObjectWrapper*>(objs[i]);
    cow->setWorldTransform(toBt(cow->m_link->GetTransform()));
  }


}


#if 0
virtual void BodyVsAll(const KinBody& body, const CollisionPairIgnorer* ignorer, vector<Collision>& collisions) {
  vector<Collision> allcollisions;
  AllVsAll(ignorer, allcollisions);
  const KinBody* pbody = &body;
  BOOST_FOREACH(Collision& col, allcollisions) {
    if (col.linkA->GetParent().get() == pbody || col.linkB->GetParent().get() == pbody) {
      collisions.push_back(col);
    }
  }
}
#endif

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



void CheckShapeCast(btCollisionShape* shape, const btTransform& tf0, const btTransform& tf1,
    KinBody::Link* link, btCollisionWorld* world, CollisionPairIgnorer* ignorer, vector<Collision>& collisions) {
  if (btConvexShape* convex = dynamic_cast<btConvexShape*>(shape)) {
    CastHullShape* shape = new CastHullShape(convex, tf0.inverseTimes(tf1));
    CollisionObjectWrapper* obj = new CollisionObjectWrapper(link);
    obj->setCollisionShape(shape);
    obj->setWorldTransform(tf0);
    CollisionCollector cc(collisions, *ignorer, *link);
    cc.m_collisionFilterMask = KinBodyFilter;
    cc.m_collisionFilterGroup = RobotFilter;
    world->contactTest(obj, cc);
    delete obj;
    delete shape;
  }
  else if (btCompoundShape* compound = dynamic_cast<btCompoundShape*>(shape)) {
    for (int i = 0; i < compound->getNumChildShapes(); ++i) {
      CheckShapeCast(compound->getChildShape(i), tf0*compound->getChildTransform(i), tf1*compound->getChildTransform(i), link, world, ignorer, collisions);
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
    CheckShapeCast(m_link2cow[links[i].get()]->getCollisionShape(), tbefore[i], tafter[i], links[i].get(), m_world, &m_ignorer, collisions);
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
