#include <boost/python.hpp>
#include "sim_kinect.hpp"
#include "macros.h"
#include "numpy_utils.hpp"
#include "osgviewer/osgviewer.hpp"
using namespace Eigen;
using namespace OpenRAVE;
using std::vector;

namespace py = boost::python;

py::object openravepy;

EnvironmentBasePtr GetCppEnv(py::object py_env) {
  int id = py::extract<int>(openravepy.attr("RaveGetEnvironmentId")(py_env));
  EnvironmentBasePtr cpp_env = RaveGetEnvironment(id);
  return cpp_env;
}

// class TRAJOPT_API FakeKinect {
// public:
//   FakeKinect(osg::Group* root);
//   void SetPose(const OpenRAVE::Transform& pose);
//   void SetIntrinsics(float f);
//   void Update();
//   float* GetDepthImage();
//   RGB* GetColorImage();
// 
//   osg::ref_ptr<osg::Camera> m_cam;
//   osgViewer::Viewer m_viewer;
//   osg::ref_ptr<KinectCallback> m_cb;
//   OpenRAVE::Transform m_tf;
//   float m_f;
// };

boost::shared_ptr<FakeKinect> pyCreateFakeKinect(py::object py_env) {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(GetCppEnv(py_env));
  return boost::shared_ptr<FakeKinect>(new FakeKinect(viewer->m_root.get()));
}

py::object FakeKinect_GetColorImage(FakeKinect* fk) {
  return toNdarray3<unsigned char>(fk->GetColorImage(), 480, 640, 3);  
}

py::object FakeKinect_GetDepthImage(FakeKinect* fk) {
  return toNdarray2<float>(fk->GetDepthImage(), 480, 640);
}

void FakeKinect_SetPose(FakeKinect* fk, py::object xyz, py::object wxyz) {
  OpenRAVE::Transform T;
  for (int i=0; i < 3; ++i) T.trans[i] = py::extract<float>(xyz[i]);
  for (int i=0; i < 4; ++i) T.rot[i] = py::extract<float>(wxyz[i]);
  fk->SetPose(T);
}


BOOST_PYTHON_MODULE(sensorsimpy) {


  np_mod = py::import("numpy");
  openravepy = py::import ("openravepy");
  py::def("CreateFakeKinect", &pyCreateFakeKinect);
  py::class_<FakeKinect, boost::shared_ptr<FakeKinect> >("FakeKinect", py::no_init)
      .def("Update", &FakeKinect::Update)
      .def("SetPose", &FakeKinect_SetPose)
      .def("SetIntrinsics", &FakeKinect::SetIntrinsics)
      .def("GetColorImage", &FakeKinect_GetColorImage)
      .def("GetDepthImage", &FakeKinect_GetDepthImage)
      ;
  
  
}
