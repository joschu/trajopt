#pragma once
#include <openrave/openrave.h>
#include <osg/Geometry>
#include <osgViewer/Viewer>
#include <Eigen/Core>
#include "macros.h"

struct RGB {
  unsigned char r,g,b;
};


class KinectCallback : public osg::Camera::DrawCallback {

public:

  osg::ref_ptr<osg::Image> m_depthbufferimg;
  osg::ref_ptr<osg::Image> m_colorbufferimg;
  Eigen::MatrixXf m_depthimg;

  KinectCallback( osg::Camera* camera ) ;
  void operator () ( osg::RenderInfo& info ) const;
};

class TRAJOPT_API FakeKinect {
public:
  FakeKinect(osg::Group* root);
  void SetPose(const OpenRAVE::Transform& pose);
  void SetIntrinsics(float f);
  void Update();
  float* GetDepthImage();
  unsigned char* GetColorImage(); // XXX this is bad if the row size is not a multiple of 4

  osg::ref_ptr<osg::Camera> m_cam;
  osgViewer::Viewer m_viewer;
  osg::ref_ptr<KinectCallback> m_cb;
  OpenRAVE::Transform m_tf;
  float m_f;
};
