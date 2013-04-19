#include "sim_kinect.hpp"
#include <iostream>
#include "utils/logging.hpp"
#include <osg/io_utils>
using namespace std;


void ComputeDepthImage( osg::Camera* camera, osg::ref_ptr<osg::Image> depthbufferimg, Eigen::MatrixXf& depthimage) {
  LOG_DEBUG("computing depth image");
  const osg::Viewport* viewport = camera->getViewport();
  size_t width = viewport->width();
  size_t height = viewport->height();


  depthimage.resize( height, width);
  float* z = (float*)depthbufferimg->data();

  // get the intrinsic parameters of the camera
  double fovy, aspectRatio, Zn, Zf;
  camera->getProjectionMatrixAsPerspective( fovy, aspectRatio, Zn, Zf );

  for( int r=0; r < height; ++r ){
    for( int c=0; c < width; ++c ){
      depthimage(height-r-1,c) =  Zn*Zf / (Zf - z[ r*width + c ]*(Zf-Zn));
    }
  }
}

KinectCallback::KinectCallback( osg::Camera* camera ) {
  const osg::Viewport* viewport    = camera->getViewport();
  osg::Viewport::value_type width  = viewport->width();
  osg::Viewport::value_type height = viewport->height();
  m_depthbufferimg = new osg::Image;
  m_depthbufferimg->allocateImage(width, height, 1, GL_DEPTH_COMPONENT, GL_FLOAT);
  m_colorbufferimg = new osg::Image;
  m_colorbufferimg->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
  camera->attach(osg::Camera::DEPTH_BUFFER, m_depthbufferimg.get(), 0, 0);
  camera->attach(osg::Camera::COLOR_BUFFER, m_colorbufferimg.get(), 0, 0);
}

void KinectCallback::operator () ( osg::RenderInfo& info ) const  {
  osg::Camera* camera = info.getCurrentCamera();


  KinectCallback* this2 = const_cast<KinectCallback*>(this);
  ComputeDepthImage(camera, m_depthbufferimg, this2->m_depthimg);

}



void GLParamsFromCamIntrinsics(float width, float height, float fx, float fy, float& fovy, float& aspect) {
  fovy = 2*atan(.5*height/fy)*180/M_PI;
  aspect = (width*fy) / (height*fx);
}


FakeKinect::FakeKinect(osg::Group* root)  {

  const int width = 640, height = 480;

  // set up view
  m_viewer.setSceneData(root);
  m_viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);

  m_cam = m_viewer.getCamera();

  SetIntrinsics(525);
  SetPose(OpenRAVE::Transform());

  // set up pixel buffer
  osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
  traits->x = 0;
  traits->y = 0;
  traits->width = width;
  traits->height = height;
  traits->red = 8;
  traits->green = 8;
  traits->blue = 8;
  traits->alpha = 8;
  traits->windowDecoration = false;
  traits->pbuffer = true;
  traits->sharedContext = 0;
  osg::ref_ptr<osg::GraphicsContext> pbuffer(osg::GraphicsContext::createGraphicsContext(traits.get()));
  assert(pbuffer.valid());
  m_cam->setGraphicsContext(pbuffer.get());
  m_cam->setViewport(new osg::Viewport(0, 0, width, height));

  m_cb = new KinectCallback(m_cam.get());
  m_cam->setFinalDrawCallback(m_cb.get());

  m_viewer.realize();
}

void FakeKinect::SetIntrinsics(float f) {
  float fovy, aspect;
  float width=640, height=480;
  GLParamsFromCamIntrinsics(width, height, f, f, fovy, aspect);
  m_cam->setProjectionMatrixAsPerspective(fovy, aspect, .3, 10);
}

void FakeKinect::SetPose(const OpenRAVE::Transform& T) {
//  osg::Matrix m;
//  m.setRotate(osg::Quat(T.rot[1], T.rot[2], T.rot[3], T.rot[0]));
//  m.setTrans(osg::Vec3f(T.trans[0], T.trans[1], T.trans[2]));
//  m_cam->setViewMatrix(m);
  OpenRAVE::TransformMatrix M = T;
  osg::Vec3f eye(T.trans[0], T.trans[1], T.trans[2]);
  osg::Vec3f center(M.rot(0,2), M.rot(1,2), M.rot(2,2));
  center += eye;
  osg::Vec3f up(-M.rot(0,1), -M.rot(1,1), -M.rot(2,1));
  m_cam->setViewMatrixAsLookAt(eye, center, up);
//  cout << "eye: " << eye << " center: " << center << " up: " << up << endl;
}

void FakeKinect::Update() {
  m_viewer.frame();
}

float* FakeKinect::GetDepthImage() {
  return m_cb->m_depthimg.data();
}

unsigned char* FakeKinect::GetColorImage() {
  return m_cb->m_colorbufferimg->data();
}
