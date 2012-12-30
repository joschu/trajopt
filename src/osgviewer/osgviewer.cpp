#include "osgviewer.h"
#include <boost/foreach.hpp>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <osg/Material>
#include <osgUtil/SmoothingVisitor>
#include <cstdio>
#include <algorithm>
#include <osg/Array>
#include <osg/BlendFunc>

using namespace osg;
using namespace OpenRAVE;
using namespace std;

namespace {

osg::Matrix asOsgMatrix(const OpenRAVE::Transform& T) {
  osg::Matrix m;
  m.setTrans(osg::Vec3(T.trans.x, T.trans.y, T.trans.z));
  m.setRotate(osg::Vec4(T.rot[1], T.rot[2], T.rot[3], T.rot[0]));
  return m;
}
template <class T>
osg::Vec3 toOsgVec3(const RaveVector<T>& v) {
  return Vec3(v.x, v.y, v.z);
}
template <class T>
osg::Vec4 toOsgVec4(const RaveVector<T>& v) {
  return Vec4(v.x, v.y, v.z, v.w);
}

osg::Drawable* toOsgDrawable(const KinBody::Link::TRIMESH& mesh) {

  osg::Vec3Array* vec = new osg::Vec3Array();
  vec->resize( mesh.vertices.size());
  for(int i = 0; i < mesh.vertices.size(); ++i) {
    const Vector& v = mesh.vertices[i];
    (*vec)[i].set( v.x, v.y, v.z );
  }

  osg::DrawElementsUInt* deui = new osg::DrawElementsUInt( GL_TRIANGLES );
  for(int i = 0; i < mesh.indices.size(); ++i)
    deui->push_back( mesh.indices[ i ] );


  osg::Vec4Array* color = new osg::Vec4Array();
  color->push_back( osg::Vec4( 1., 1., 1., 1. ) );

  osg::Geometry* geom = new osg::Geometry;
  geom->setVertexArray( vec );
  geom->setColorArray( color );
  geom->setColorBinding( osg::Geometry::BIND_OVERALL );

  geom->addPrimitiveSet( deui );
  return geom;
}

Node* osgNodeFromGeom(const KinBody::Link::Geometry& geom) {
  osg::Geode* geode = new osg::Geode;

  switch(geom.GetType()) {

  case KinBody::Link::GEOMPROPERTIES::GeomSphere: {

    osg::Sphere* s = new osg::Sphere();
    s->setRadius(geom.GetSphereRadius());
    osg::ShapeDrawable* sd = new osg::ShapeDrawable(s);
    geode->addDrawable(sd);
    break;
  }
  //  Geometry is defined like a Box
  case KinBody::Link::GEOMPROPERTIES::GeomBox: {

    osg::Box* box = new osg::Box();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    OpenRAVE::Vector v = geom.GetBoxExtents();
    box->setHalfLengths(osg::Vec3(v.x,v.y,v.z));
    osg::ShapeDrawable* sd = new osg::ShapeDrawable(box);
    geode->addDrawable(sd);
    break;
  }
  //  Geometry is defined like a Cylinder
  case KinBody::Link::GEOMPROPERTIES::GeomCylinder: {


    // make SoCylinder point towards z, not y
    osg::Cylinder* cy = new osg::Cylinder();
    cy->setRadius(geom.GetCylinderRadius());
    cy->setHeight(geom.GetCylinderHeight());
    osg::ShapeDrawable* sd = new osg::ShapeDrawable(cy);
    geode->addDrawable(sd);
    break;
  }
  //  Extract geometry from collision Mesh
  case KinBody::Link::GEOMPROPERTIES::GeomTrimesh: {
    // make triangleMesh
    osg::Drawable* mesh_drawable = toOsgDrawable(geom.GetCollisionMesh());
    geode->addDrawable(mesh_drawable);
    break;
  }
  default:
    RAVELOG_ERROR("don't know how to make an osg node for geometry of type %i", geom.GetType());
    break;
  }

  osg::StateSet* state = geode->getOrCreateStateSet();
  osg::Material* mat = new osg::Material;
  OpenRAVE::Vector diffuse = geom.GetDiffuseColor();
  mat->setDiffuse( osg::Material::FRONT_AND_BACK, osg::Vec4(diffuse.x,diffuse.y,diffuse.z,1) );
  OpenRAVE::Vector amb = geom.GetDiffuseColor();
  mat->setAmbient( osg::Material::FRONT_AND_BACK, osg::Vec4(amb.x,amb.y,amb.z,1) );
  mat->setTransparency(osg::Material::FRONT_AND_BACK,geom.GetTransparency());
  state->setAttribute(mat);

  osgUtil::SmoothingVisitor sv;
  geode->accept(sv);

  return geode;
}
MatrixTransform* osgNodeFromLink(const KinBody::Link& link) {
  /* each geom is a child */
  osg::MatrixTransform* link_node = new osg::MatrixTransform;
  const vector<KinBody::Link::GeometryPtr>& geoms = link.GetGeometries();
  for (int i=0; i < geoms.size(); ++i) {

    osg::Node* geom_node = osgNodeFromGeom(*geoms[i]);

    osg::Matrix m = asOsgMatrix( geoms[i]->GetTransform() );
    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(m);
    mt->addChild(geom_node);

    link_node->addChild(geom_node);
  }
  link_node->setMatrix(Matrix::identity());
  return link_node;
}

class KinBodyGroup : public osg::Group {
public:
  vector<KinBody::LinkPtr> links; // links with geometry
  vector<osg::MatrixTransform*> nodes; // corresponding nodes
  void update() {
    for (int i=0; i < links.size(); ++i) {
      nodes[i]->setMatrix(asOsgMatrix(links[i]->GetTransform()));
    }
  }

};


KinBodyGroup* osgNodeFromKinBody(const KinBody& body) {
  /* each link is a child */
  KinBodyGroup* group = new KinBodyGroup;
  const vector<KinBody::LinkPtr>& links = body.GetLinks();
  for (int i=0; i < links.size(); ++i) {
    if (links[i]->GetGeometries().size() > 0) {
      MatrixTransform* link_node = osgNodeFromLink(*links[i]);
      group->addChild(link_node);
      group->links.push_back(links[i]);
      group->nodes.push_back(link_node);
    }
  }

  return group;
}



void AddLights(osg::Group* group) {
  {
    osg::Light* light = new osg::Light;
    light->setLightNum(0);
    light->setPosition(osg::Vec4(-4,0,4,1));
    osg::LightSource* lightSource = new osg::LightSource;
    lightSource->setLight(light);
    light->setDiffuse(osg::Vec4(1,.9,.9,1)*1);
    light->setConstantAttenuation(0);
    light->setLinearAttenuation(.25);
    group->addChild(lightSource);
    group->getOrCreateStateSet()->setMode(GL_LIGHT0, osg::StateAttribute::ON);
  }

  {
    osg::Light* light = new osg::Light;
    light->setLightNum(1);
    light->setPosition(osg::Vec4(4,0,4,1));
    osg::ref_ptr<osg::LightSource> lightSource = new osg::LightSource;
    lightSource->setLight(light);
    light->setDiffuse(osg::Vec4(.9,.9,1,1)*1);
    light->setConstantAttenuation(0);
    light->setLinearAttenuation(.25);
    group->addChild(lightSource.get());
    group->getOrCreateStateSet()->setMode(GL_LIGHT1, osg::StateAttribute::ON);
  }

}


// http://forum.openscenegraph.org/viewtopic.php?t=7806
void   AddCylinderBetweenPoints(osg::Vec3   StartPoint, osg::Vec3   EndPoint, float radius, osg::Vec4   CylinderColor, osg::Group   *pAddToThisGroup)
{
    osg::Vec3   center;
    float      height;

   osg::ref_ptr<osg::Cylinder> cylinder;
   osg::ref_ptr<osg::ShapeDrawable> cylinderDrawable;
   osg::ref_ptr<osg::Material> pMaterial;
   osg::ref_ptr<osg::Geode> geode;

   height = (StartPoint- EndPoint).length();
    center = osg::Vec3( (StartPoint.x() + EndPoint.x()) / 2,  (StartPoint.y() + EndPoint.y()) / 2,  (StartPoint.z() + EndPoint.z()) / 2);

   // This is the default direction for the cylinders to face in OpenGL
   osg::Vec3   z = osg::Vec3(0,0,1);

   // Get diff between two points you want cylinder along
   osg::Vec3 p = (StartPoint - EndPoint);

   // Get CROSS product (the axis of rotation)
   osg::Vec3   t = z ^  p;

   // Get angle. length is magnitude of the vector
   double angle = acos( (z * p) / p.length());

   //   Create a cylinder between the two points with the given radius
    cylinder = new osg::Cylinder(center,radius,height);
   cylinder->setRotation(osg::Quat(angle, osg::Vec3(t.x(), t.y(), t.z())));

   //   A geode to hold our cylinder
   geode = new osg::Geode;
   cylinderDrawable = new osg::ShapeDrawable(cylinder );
    geode->addDrawable(cylinderDrawable);

   //   Set the color of the cylinder that extends between the two points.
   pMaterial = new osg::Material;
   pMaterial->setDiffuse( osg::Material::FRONT, CylinderColor);
   geode->getOrCreateStateSet()->setAttribute( pMaterial, osg::StateAttribute::OVERRIDE );

   //   Add the cylinder between the two points to an existing group
   pAddToThisGroup->addChild(geode);
}

class SetColorsVisitor : public osg::NodeVisitor
{
public:
  osg::Vec4 color;
  SetColorsVisitor(const osg::Vec4& _color) : color(_color) {
    setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
  }
  void apply( osg::Geode& geode ) {
    StateSet* ss = geode.getOrCreateStateSet();
    osg::Material* mat = static_cast<Material*>(ss->getAttribute(StateAttribute::MATERIAL));
    mat->setAmbient(osg::Material::FRONT_AND_BACK, color);
    mat->setDiffuse(osg::Material::FRONT_AND_BACK, color);
    if (color[3] < 1) {
      ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) );
      ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
    }
  }
};
class SetTransparencyVisitor : public osg::NodeVisitor
{
public:
  float alpha;
  SetTransparencyVisitor(float _alpha) : alpha(_alpha) {
    setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN );
  }
  void apply( osg::Geode& geode ) {
    StateSet* ss = geode.getOrCreateStateSet();
    osg::Material* mat = static_cast<Material*>(ss->getAttribute(StateAttribute::MATERIAL));
    // mat->setTransparency(osg::Material::FRONT_AND_BACK, alpha);
    // for some reason setTransparency doesn't work so well
    osg::Vec4 amb = mat->getAmbient(Material::FRONT_AND_BACK);
    amb[3] = alpha;
    mat->setAmbient(Material::FRONT_AND_BACK,amb);
    osg::Vec4 diffuse = mat->getDiffuse(Material::FRONT_AND_BACK);
    diffuse[3] = alpha;
    mat->setDiffuse(Material::FRONT_AND_BACK, diffuse);
    if (alpha < 1) {
      ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) );
      ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
    }
    else ss->setRenderingHint(StateSet::OPAQUE_BIN);

  }
};


class RefPtrHolder : public UserData {
public:
  osg::ref_ptr<osg::Referenced> rp;
  RefPtrHolder(osg::Referenced* x) {
    rp = x;
  }
};


class OsgGraphHandle : public OpenRAVE::GraphHandle {
public:
  osg::Group* parent;
  osg::ref_ptr<osg::Node> node;
  OsgGraphHandle(osg::Node* _node, osg::Group* _parent) : node(_node), parent(_parent) {
    parent->addChild(node);
  }
  ~OsgGraphHandle() {
    parent->removeChild(node);
  }
};

}



bool OSGViewer::EventHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa) {
    bool suppressDefault = false;
    osgGA::GUIEventAdapter::EventType t = ea.getEventType();
    // keypress handlers
    if (t == osgGA::GUIEventAdapter::KEYDOWN) {
      int key = ea.getKey();
      KeyCallbackMap::iterator it = key_cbs.find(key);
      if (it != key_cbs.end()) (it->second)(ea);
    }

    // general event handlers
    for (int i=0; i < event_cbs.size(); ++i) suppressDefault |= event_cbs[i](ea);


    if (suppressDefault) return false;
    else return osgGA::TrackballManipulator::handle(ea, aa);
}



OSGViewer::OSGViewer(EnvironmentBasePtr env) : ViewerBase(env), m_idling(false) {

  m_root = new Group;
//  m_viewer = new osgViewer::Viewer();
  m_viewer.setSceneData(m_root.get());
  m_viewer.setUpViewInWindow(0, 0, 640, 480);
  m_viewer.realize();
  m_cam = m_viewer.getCamera();
  m_handler = new EventHandler;
  m_viewer.setCameraManipulator(m_handler.get());
  AddLights(m_root);
  m_cam->setClearColor(osg::Vec4(1,1,1,1));

  AddKeyCallback('h', boost::bind(&OSGViewer::PrintHelp, this), "Display help");
  AddKeyCallback('p', boost::bind(&OSGViewer::Idle, this), "Toggle idle");
  PrintHelp();
}

int OSGViewer::main(bool bShow) {
  UpdateSceneData();
  return m_viewer.run();
}

void OSGViewer::Idle() {
  UpdateSceneData();
  if (m_idling) { // stop idling
    m_idling = false;
    return;
  }
  else { // start idling
    m_idling = true;
    RAVELOG_INFO("press p to stop idling\n");
    while (!m_viewer.done() && m_idling) {
      m_viewer.frame();
    }
  }
}

void OSGViewer::Draw() {
  UpdateSceneData();
  m_viewer.frame();
}

void OSGViewer::RemoveKinBody(OpenRAVE::KinBodyPtr pbody) {
  KinBodyGroup* node = (KinBodyGroup*)pbody->GetUserData("osg").get();
  m_root->removeChild(node);
  pbody->RemoveUserData("osg");
}

KinBodyGroup* GetOsgGroup(KinBody& body) {
  UserDataPtr rph = body.GetUserData("osg");
  return rph ? static_cast<KinBodyGroup*>(static_cast<RefPtrHolder*>(rph.get())->rp.get())
      : NULL;
}
KinBodyGroup* CreateOsgGroup(KinBody& body) {
  assert(!body.GetUserData("osg"));
  RAVELOG_DEBUG("creating graphics for kinbody %s\n", body.GetName().c_str());
  osg::Node* node = osgNodeFromKinBody(body);
  UserDataPtr rph = UserDataPtr(new RefPtrHolder(node));
  body.SetUserData("osg", rph);
  return static_cast<KinBodyGroup*>(static_cast<RefPtrHolder*>(rph.get())->rp.get());
}


OSGViewer::~OSGViewer(){
}


void OSGViewer::UpdateSceneData() {
  vector<OpenRAVE::KinBodyPtr> bodies;
  GetEnv()->GetBodies(bodies);
  for (int i=0; i < bodies.size(); ++i) {
    KinBody& body = *bodies[i];
    KinBodyGroup* group = GetOsgGroup(body);
    if (!group) {
      group = CreateOsgGroup(body);
      m_root->addChild(group);
    }
    group->update();
  }
}

void OSGViewer::AddMouseCallback(const MouseCallback& cb) {
  m_handler->event_cbs.push_back(cb);
  // todo: print help
}
void OSGViewer::AddKeyCallback(int key, const KeyCallback& cb, const std::string& help) {
  EventHandler::KeyCallbackMap::iterator it = m_handler->key_cbs.find(key);
  if (it==m_handler->key_cbs.end()) {
    m_handler->key_cbs[key] = cb;
    m_handler->descs[key] = help;
  }
  else RAVELOG_ERROR("error: key %c was already bound!\n", key);

}


void OSGViewer::PrintHelp() {
  stringstream ss;
  ss << "====== Welcome to the OSG Viewer =======\n"
  "Mouse: \n"
  "- rotate by holding left mouse button and dragging\n"
  "- zoom by holding right mouse button and dragging\n"
  "key bindings:\n"
  "Esc       quit application\n"
  "Space     reset view\n";
  for (EventHandler::Key2Desc::iterator it = m_handler->descs.begin(); it != m_handler->descs.end(); ++it) {
    ss << boost::format("%c:       %s\n")%((char)it->first)%(it->second);
  }
  RAVELOG_INFO(ss.str().c_str());
}


void SetColor(GraphHandlePtr handle, const osg::Vec4& color) {
  if (OsgGraphHandle* osghandle = dynamic_cast<OsgGraphHandle*>(handle.get())) {
    SetColorsVisitor visitor(color);
    osghandle->node->accept(visitor);
  }
}
void SetTransparency(GraphHandlePtr handle, float alpha) {
  if (OsgGraphHandle* osghandle = dynamic_cast<OsgGraphHandle*>(handle.get())) {
    SetTransparencyVisitor visitor(alpha);
    osghandle->node->accept(visitor);
  }
}

OpenRAVE::GraphHandlePtr OSGViewer::drawarrow(const RaveVectorf& p1, const RaveVectorf& p2, float fwidth, const RaveVectorf& color) {
  osg::Group* group = new osg::Group;
  AddCylinderBetweenPoints(toOsgVec3(p1), toOsgVec3(p2), fwidth, toOsgVec4(color), group);
  return GraphHandlePtr(new OsgGraphHandle(group, m_root.get()));
}


GraphHandlePtr OSGViewer::PlotKinBody(KinBodyPtr body) {
  KinBodyGroup* orig = GetOsgGroup(*body);
  /* Note: we could easily plot a kinbody that's not part of the environment, but
    there would be a problem if you plot something and then add it later
   */
  OPENRAVE_ASSERT_FORMAT0(orig != NULL, "kinbody not part of scene graph", ORE_Assert);
  orig->update();
  osg::Group* copy = new osg::Group(*orig, CopyOp(CopyOp::DEEP_COPY_NODES | CopyOp::DEEP_COPY_STATESETS | CopyOp::DEEP_COPY_STATEATTRIBUTES));
  OsgGraphHandle* handle = new OsgGraphHandle(copy, m_root.get());
  return GraphHandlePtr(handle);
}

GraphHandlePtr OSGViewer::PlotLink(KinBody::LinkPtr link) {
  KinBodyPtr body = link->GetParent();
  KinBodyGroup* orig = GetOsgGroup(*body);
  OPENRAVE_ASSERT_FORMAT0(orig != NULL, "kinbody not part of scene graph", ORE_Assert);
  orig->update();

  vector<KinBody::LinkPtr>::iterator it = std::find(orig->links.begin(), orig->links.end(), link);
  if (it == orig->links.end()) {
    RAVELOG_ERROR("want to plot link %s, can't find it\n", link->GetName().c_str());
    return GraphHandlePtr();
  }

  osg::Node* orig_node = orig->nodes[it - orig->links.begin()];
  osg::Node* copy = static_cast<Group*>(orig_node->clone(CopyOp(CopyOp::DEEP_COPY_NODES)));
  OsgGraphHandle* handle = new OsgGraphHandle(copy, m_root.get());
  return GraphHandlePtr(handle);
}

GraphHandlePtr OSGViewer::PlotAxes(const OpenRAVE::Transform& T, float size) {
  osg::Matrix m = asOsgMatrix(T);
  osg::Group* group = new osg::Group;
  osg::Vec3 o = toOsgVec3(T.trans);
  osg::Vec3 x = o + Vec3(m(0,0), m(1,0), m(2,0))*size;
  osg::Vec3 y = o + Vec3(m(0,1), m(1,1), m(2,1))*size;
  osg::Vec3 z = o + Vec3(m(0,2), m(1,2), m(2,2))*size;
  AddCylinderBetweenPoints(o, x, size/10, osg::Vec4(1,0,0,1), group);
  AddCylinderBetweenPoints(o, y, size/10, osg::Vec4(0,1,0,1), group);
  AddCylinderBetweenPoints(o, z, size/10, osg::Vec4(0,0,1,1), group);
  return GraphHandlePtr(new OsgGraphHandle(group, m_root.get()));
}

