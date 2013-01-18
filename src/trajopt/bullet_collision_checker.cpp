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
using namespace util;
using namespace std;
using namespace trajopt;

namespace {

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
//    if ( m_ignorer.CanCollide(*linkA, *linkB)) {
    if (true) {
      m_collisions.push_back(Collision(linkA, linkB, toOR(cp.m_positionWorldOnA), toOR(cp.m_positionWorldOnB),
          toOR(cp.m_normalWorldOnB), cp.m_distance1));
      RAVELOG_DEBUG("collide %s-%s\n", linkA->GetName().c_str(), linkB->GetName().c_str());
    }
    return 0;
  }
  bool needsCollision(btBroadphaseProxy* proxy0) const {
      KinBody::Link* otherlink = static_cast<CollisionObjectWrapper*>(proxy0->m_clientObject)->m_link;
      bool out = m_ignorer.CanCollide(m_link, *otherlink);
      return out;
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
  void AllVsAll(vector<Collision>& collisions);
  virtual void LinksVsAll(const vector<KinBody::LinkPtr>& links, vector<Collision>& collisions);
  virtual void LinkVsAll(const KinBody::Link& link, vector<Collision>& collisions);
  virtual void UpdateBulletFromRave();
  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>& handles);
  virtual void ContinuousCheckTrajectory(const TrajArray& traj, RobotAndDOF& rad, vector<Collision>&);
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


struct KinBodyCollisionData : public OpenRAVE::UserData {
  OpenRAVE::KinBodyWeakPtr body;
  std::vector<KinBody::LinkWeakPtr> links;
  std::vector<COWPtr> cows;
  KinBodyCollisionData(OR::KinBodyPtr _body) : body(_body) {}
};
typedef boost::shared_ptr<KinBodyCollisionData> CDPtr;

void BulletCollisionChecker::AddKinBody(const OR::KinBodyPtr& body) {
  CDPtr cd(new KinBodyCollisionData(body));

  const vector<OR::KinBody::LinkPtr> links = body->GetLinks();

  body->SetUserData("bt", cd);
  BOOST_FOREACH(const OR::KinBody::LinkPtr& link, links) {
    if (link->GetGeometries().size() > 0) {
      COWPtr new_cow = CollisionObjectFromLink(link);
      if (new_cow) {
        SetCow(link.get(), new_cow.get());
        m_world->addCollisionObject(new_cow.get());
        new_cow->setContactProcessingThreshold(m_contactDistance);
        RAVELOG_DEBUG("added collision object for  link %s\n", link->GetName().c_str());
        cd->links.push_back(link);
        cd->cows.push_back(new_cow);
      }
      else {
        RAVELOG_WARN("ignoring link %s\n", link->GetName().c_str());
      }
    }
  }
  body->SetUserData("bt", cd);

//  const std::set<int>& pairhashes = body->GetAdjacentLinks();
//  BOOST_FOREACH(const int& p, pairhashes) {
//    RAVELOG_DEBUG("excluding pair %s %s\n", links[p & 0xffff]->GetName().c_str(), links[p >> 16]->GetName().c_str());
//    m_ignorer.ExcludePair(*links[p & 0xffff], *links[p >> 16]);
//  }
}
void BulletCollisionChecker::RemoveKinBody(const OR::KinBodyPtr& body) {

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

vector<btTransform> rightMultiplyAll(const vector<btTransform>& xs, const btTransform& y) {
  vector<btTransform> out(xs.size());
  for (int i=0; i < xs.size(); ++i) out[i] = xs[i]*y;
  return out;
}

void ContinuousCheckShape(btCollisionShape* shape, const vector<btTransform>& transforms,
    KinBody::Link* link, btCollisionWorld* world, vector<Collision>& collisions) {
  if (btConvexShape* convex = dynamic_cast<btConvexShape*>(shape)) {
    for (int i=0; i < transforms.size()-1; ++i) {
      btCollisionWorld::ClosestConvexResultCallback ccc(btVector3(NAN, NAN, NAN), btVector3(NAN, NAN, NAN));
      world->convexSweepTest(convex, transforms[i], transforms[i+1], ccc, 0);
      if (ccc.hasHit()) {
        collisions.push_back(Collision(link, getLink(ccc.m_hitCollisionObject),
            toOR(ccc.m_hitPointWorld), toOR(ccc.m_hitPointWorld), toOR(ccc.m_hitNormalWorld), 0, i+ccc.m_closestHitFraction));
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
  // first calculate transforms of all the relevant links at each step
  vector<KinBody::LinkPtr> links;
  vector<int> link_inds;
  rad.GetAffectedLinks(links, true, link_inds);

  // remove them, because we can't check moving stuff against each other
  vector<CollisionObjectWrapper*> cows;
  BOOST_FOREACH(KinBody::LinkPtr& link, links) {
    CollisionObjectWrapper* cow = GetCow(link.get());
    cows.push_back(cow);
    m_world->removeCollisionObject(cow);
  }

  typedef vector<btTransform> TransformVec;
  vector<TransformVec> link2transforms(links.size(), TransformVec(traj.rows()-1));

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

  // add them back
  BOOST_FOREACH(CollisionObjectWrapper* cow, cows) {
    m_world->addCollisionObject(cow);
  }



}







}







namespace trajopt {



CollisionCheckerPtr CreateCollisionChecker(OR::EnvironmentBaseConstPtr env) {
  CollisionCheckerPtr checker(new BulletCollisionChecker(env));
  return checker;
}
}
