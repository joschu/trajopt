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

const float MARGIN = 0;

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
  if ( gPairIgnorer != NULL && !gPairIgnorer->CanCollide(
      *(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy0->m_clientObject)->m_link),
      *(static_cast<CollisionObjectWrapper*>(collisionPair.m_pProxy1->m_clientObject)->m_link)))
    return;
  else {
    dispatcher.defaultNearCallback(collisionPair, dispatcher, dispatchInfo);}
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

    for (size_t i = 0; i < mesh.indices.size(); i += 3)
      ptrimesh.addTriangle(toBt(mesh.vertices[i]), toBt(mesh.vertices[i + 1]),
          toBt(mesh.vertices[i + 2]));
    // store the trimesh somewhere so it doesn't get deallocated by the smart pointer

    btConvexTriangleMeshShape convexTrimesh(&ptrimesh);
    convexTrimesh.setMargin(MARGIN); // margin: hull padding

    //Create a hull shape to approximate Trimesh
    btShapeHull shapeHull(&convexTrimesh);
    shapeHull.buildHull(-666); // note: margin argument not used

    btConvexHullShape *convexShape = new btConvexHullShape();
    for (int i = 0; i < shapeHull.numVertices(); ++i)
      convexShape->addPoint(shapeHull.getVertexPointer()[i]);

    subshape = convexShape;
    break;
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

      assert(subshape != NULL);
      cow->manage(subshape);
      subshape->setMargin(MARGIN);
      btTransform geomTrans = toBt(geom->GetTransform());
      compound->addChildShape(geomTrans, subshape);

    }

  }

  cow->setWorldTransform(toBt(link->GetTransform()));

  return cow;
}



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

#if 0
    btConvexPolyhedron* poly = const_cast<btConvexPolyhedron*>(convex->getConvexPolyhedron());
    vector<btVector3> points;
    poly->initialize();
    points.reserve(poly->m_faces.size() * 3);
    for (int iFace = 0; iFace < poly->m_faces.size(); ++iFace) {
      btFace& face = poly->m_faces[iFace];
      for (int iPolygon = 2; iPolygon < face.m_indices.size(); ++iPolygon) {
        int polyinds[3] = { face.m_indices[iPolygon], face.m_indices[iPolygon
                                                                     - 1], face.m_indices[0] };
        points.push_back(tf * poly->m_vertices[polyinds[0]]);
        points.push_back(tf * poly->m_vertices[polyinds[1]]);
        points.push_back(tf * poly->m_vertices[polyinds[2]]);
      }
    }
    handles.push_back(env.drawtrimesh((float*) points.data(), sizeof(btVector3), NULL,
        points.size() / 3, OR::RaveVector<float>(0, 1, 0, .25)));
#endif
    btShapeHull shapeHull(convex);
    shapeHull.buildHull(-666); // note: margin argument not used
    RAVELOG_INFO("rendering convex shape with %i triangles\n", shapeHull.numTriangles());
    vector<btVector3> tverts(shapeHull.numVertices());
    for (int i=0; i < shapeHull.numVertices(); ++i) {
      tverts[i] = tf*shapeHull.getVertexPointer()[i];
    }
    //    for (int i = 0; i < shapeHull.numVertices(); ++i)
    //      convexShape->addPoint(shapeHull.getVertexPointer()[i]);
    handles.push_back(env.drawtrimesh((float*) tverts.data(), sizeof(btVector3), (int*)shapeHull.getIndexPointer(),
        shapeHull.numTriangles(), OR::RaveVector<float>(1,1,1,.1)));
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
  CollisionCollector(vector<Collision>& collisions, const CollisionPairIgnorer& ignorer) : m_collisions(collisions), m_ignorer(ignorer) {}
  virtual btScalar addSingleResult(btManifoldPoint& cp,
      const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,
      const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1) {
    const CollisionObjectWrapper* objA = static_cast<const CollisionObjectWrapper*>(colObj0Wrap->getCollisionObject());
    const CollisionObjectWrapper* objB = static_cast<const CollisionObjectWrapper*>(colObj1Wrap->getCollisionObject());
    const KinBody::Link* linkA = objA->m_link;
    const KinBody::Link* linkB = objB->m_link;
    if (m_ignorer.CanCollide(*linkA, *linkB)) {
      m_collisions.push_back(Collision(linkA, linkB, toOR(cp.m_positionWorldOnA), toOR(cp.m_positionWorldOnB),
          toOR(cp.m_normalWorldOnB), cp.m_distance1));
    }
    return 0;
  }
};

class BulletCollisionChecker : public CollisionChecker {
  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;
  typedef map<const OR::KinBody::Link*, COWPtr> Link2Cow;
  Link2Cow m_link2cow;

public:
  BulletCollisionChecker(OR::EnvironmentBaseConstPtr env) :
    CollisionChecker(env) {
    m_coll_config = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_coll_config);
    m_broadphase = new btDbvtBroadphase();
    m_world = new btCollisionWorld(m_dispatcher, m_broadphase, m_coll_config);
    m_dispatcher->registerCollisionCreateFunc(BOX_SHAPE_PROXYTYPE,BOX_SHAPE_PROXYTYPE,
        m_coll_config->getCollisionAlgorithmCreateFunc(CONVEX_SHAPE_PROXYTYPE, CONVEX_SHAPE_PROXYTYPE));
    m_dispatcher->setNearCallback(&nearCallback);
  }

  ~BulletCollisionChecker() {
    delete m_world;
    delete m_broadphase;
    delete m_dispatcher;
    delete m_coll_config;
  }

  void UpdateContactDistance() {
    float expansion = m_contactDistance;
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
    UpdateContactDistance();
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
        stringstream ss; ss << pt.m_localPointA << " | " << pt.m_localPointB;
        RAVELOG_DEBUG("local pts: %s\n",ss.str().c_str());
        // adjustContactPoint(pt, objA, objB);

        const KinBody::Link* bodyA = objA->m_link;
        const KinBody::Link* bodyB = objB->m_link;

        if (!ignorer || ignorer->CanCollide(*bodyA, *bodyB)) {
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


  virtual void BodyVsAll(const KinBody& body, const CollisionPairIgnorer* ignorer, vector<Collision>& collisions) {
    UpdateBulletFromRave();
    UpdateContactDistance();
    gPairIgnorer = ignorer;

    BOOST_FOREACH(const KinBody::LinkPtr& link, body.GetLinks()) {
      LinkVsAll(*link, ignorer, collisions);
    }
  }

  virtual void LinkVsAll(const KinBody::Link& link, const CollisionPairIgnorer* ignorer, vector<Collision>& collisions) {
    if (link.GetGeometries().empty()) return;
    COWPtr cow = m_link2cow[&link];
    assert(!!cow);
    CollisionCollector cc(collisions, *ignorer);
    m_world->contactTest(cow.get(), cc);
  }




  virtual void UpdateBulletFromRave() {

    //
    vector<OR::KinBodyPtr> bodies;
    m_env->GetBodies(bodies);
    BOOST_FOREACH(const OR::KinBodyPtr& body, bodies) {
      BOOST_FOREACH(const OR::KinBody::LinkPtr& link, body->GetLinks()) {
        if (link->GetGeometries().empty()) continue;
        Link2Cow::iterator it = m_link2cow.find(link.get());
        if (it == m_link2cow.end()) {
          COWPtr new_cow = CollisionObjectFromLink(link);
          if (new_cow) {
            m_link2cow[link.get()] = new_cow;
            assert(new_cow);
            m_world->addCollisionObject(new_cow.get());
            RAVELOG_DEBUG("added collision object for  link %s\n", link->GetName().c_str());
          }
          else {
            RAVELOG_DEBUG("ignoring link %s\n", link->GetName().c_str());
          }
        }
      }
    }

    RAVELOG_WARN("todo: remove objects from collision checker\n");

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

  virtual void PlotCollisionGeometry(vector<OpenRAVE::GraphHandlePtr>& handles) {
    UpdateBulletFromRave();
    btCollisionObjectArray& objs = m_world->getCollisionObjectArray();
    RAVELOG_DEBUG("%i objects in bullet world\n", objs.size());
    for (int i=0; i < objs.size(); ++i) {
      RenderCollisionShape(objs[i]->getCollisionShape(), objs[i]->getWorldTransform(), *boost::const_pointer_cast<OpenRAVE::EnvironmentBase>(m_env), handles);
    }
  }


};



}







namespace trajopt {



CollisionCheckerPtr CreateCollisionChecker(OR::EnvironmentBaseConstPtr env) {
  CollisionCheckerPtr checker(new BulletCollisionChecker(env));
  return checker;
}
}
