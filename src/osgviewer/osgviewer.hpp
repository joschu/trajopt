#pragma once
#include <openrave/openrave.h>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <boost/function.hpp>
#include <osgGA/TrackballManipulator>


typedef OpenRAVE::RaveVector<float> RaveVectorf;

class OSGViewer : public OpenRAVE::ViewerBase {
public:
  OSGViewer(OpenRAVE::EnvironmentBasePtr env);
  ~OSGViewer();
  virtual int main(bool bShow = true);
  virtual void quitmainloop() {}
  void RemoveKinBody(OpenRAVE::KinBodyPtr pbody);
  void Draw();
  void Idle();
  void UpdateSceneData();
  const std::string& GetName() const {return m_name;}


  // return false if you want to disable the default TrackballManipulator handling
  typedef boost::function<bool(const osgGA::GUIEventAdapter &)> MouseCallback;
  void AddMouseCallback(const MouseCallback& cb);
  typedef boost::function<void(const osgGA::GUIEventAdapter &)> KeyCallback;
  void AddKeyCallback(int key, const KeyCallback& cb, const std::string& help="");
  void PrintHelp();

  OpenRAVE::GraphHandlePtr drawarrow(const RaveVectorf& p1, const RaveVectorf& p2, float fwidth, const RaveVectorf& color);
  OpenRAVE::GraphHandlePtr  drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const RaveVectorf &color);
//  OpenRAVE::GraphHandlePtr drawtrimesh (const float *ppoints, int stride, const int *pIndices, int numTriangles, const boost::multi_array< float, 2 > &colors);


  void SetAllTransparency(float alpha);

  OpenRAVE::GraphHandlePtr PlotAxes(const OpenRAVE::Transform& T, float size);
  OpenRAVE::GraphHandlePtr PlotSphere(const OpenRAVE::Vector& x, float radius);

  /** copy current state of kinbody or link */
  OpenRAVE::GraphHandlePtr PlotKinBody(const OpenRAVE::KinBodyPtr);
  OpenRAVE::GraphHandlePtr PlotLink(const OpenRAVE::KinBody::LinkPtr);

  struct EventHandler : public osgGA::TrackballManipulator {
    typedef std::map<int, OSGViewer::KeyCallback> KeyCallbackMap;
    std::vector<MouseCallback> event_cbs;
    KeyCallbackMap key_cbs;
    typedef std::map<int, std::string> Key2Desc;
    Key2Desc descs; // callback descriptions
    bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
  };


  osg::ref_ptr<EventHandler> m_handler;
  private:

  osg::ref_ptr<osg::Group> m_root;
//  osg::ref_ptr<osgViewer::Viewer> m_viewer;
  osg::ref_ptr<osg::Camera> m_cam;
  osgViewer::Viewer m_viewer;
  bool m_idling;
  std::string m_name;


};
typedef boost::shared_ptr<OSGViewer> OSGViewerPtr;


void SetColor(OpenRAVE::GraphHandlePtr handle, const osg::Vec4& color);
void SetTransparency(OpenRAVE::GraphHandlePtr handle, float alpha);
