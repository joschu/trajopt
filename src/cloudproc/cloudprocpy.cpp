#include <boost/python.hpp>
#include <stdexcept>
#include <boost/python/exception_translator.hpp>
#include <boost/foreach.hpp>
#include "cloudproc.hpp"
#include "convexdecomp.hpp"
#include <iostream>
#include "utils/stl_to_string.hpp"
#include "cloudgrabber.hpp"
#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <boost/format.hpp>

using namespace Eigen;
using namespace pcl;
using namespace cloudproc;
using namespace std;
using namespace util;
namespace py = boost::python;
using boost::shared_ptr;
typedef PointCloud<PointXYZ> CloudXYZ;
typedef PointCloud<PointNormal> CloudXYZN;


template <typename T>
T* getPointer(const py::object& arr) {
  long int i = py::extract<long int>(arr.attr("ctypes").attr("data"));
  T* p = (T*)i;
  return p;
}

py::object toInt32_1d(const int* data, size_t sz) {
  py::object np_mod = py::import("numpy");
  py::object out = np_mod.attr("empty")(py::make_tuple(sz), "int32");
  int* p = getPointer<int>(out);
  memcpy(p, data, sz*sizeof(int));
  return out;
}
py::object toFloat32_2d(const float* data, int rows, int cols, int floatskip) {
  py::object np_mod = py::import("numpy");
  py::object out = np_mod.attr("empty")(py::make_tuple(rows, cols), "float32");
  const float* pin = data;
  float* pout = getPointer<float>(out);
  for (int i=0; i < rows; ++i) {
    for (int j=0; j < cols; ++j) {
      *pout = *pin;
      ++pout;
      ++pin;
    }
    pin += floatskip;
  }
  return out;
}

template <class T>
struct PyCloud {
  typedef PointCloud<T> PointCloudT;
  static py::object to2dArray(PointCloudT* cloud) {
    return toFloat32_2d((const float*)cloud->points.data(), cloud->size(), sizeof(T)/sizeof(float), 0);
  }
  static void from2dArray(PointCloudT* cloud, py::object arr) {
    py::object np_mod = py::import("numpy");
    arr = np_mod.attr("array")(arr, "float32");
    int rows = py::extract<int>(arr.attr("shape")[0]);
    int cols = py::extract<int>(arr.attr("shape")[1]);
    int cols1 = sizeof(T)/4;
    if (cols != cols1) PRINT_AND_THROW (boost::format("array must have %s columns")%cols1);
    cloud->resize(rows);
    float* p = getPointer<float>(arr);
    memcpy(&cloud->points[0], p,  rows*cols*sizeof(float));
  }
  static void save(PointCloudT* cloud, const std::string& fname) {
    saveCloud(*cloud, fname);
  }
  static int width(PointCloudT* cloud) {return cloud->width;}
  static int height(PointCloudT* cloud){return cloud->height;}
  static int size(PointCloudT* cloud){return cloud->size();}
};

py::list PyConvexDecomp(const PointCloud<PointXYZ>& cloud, float thresh) {
  vector<IntVec> hulls;
  py::list out;
  PointCloud<PointXYZ>::Ptr cloudptr = cloud.makeShared();
  ConvexDecomp1(cloudptr, thresh, NULL, &hulls);
  BOOST_FOREACH(const IntVec& inds, hulls) {
    out.append(toInt32_1d(inds.data(), inds.size()));
  }
  return out;
}

CloudXYZ::Ptr PolygonMesh_getCloud(const PolygonMesh* mesh) {
  CloudXYZ::Ptr cloud(new CloudXYZ());
  pcl::fromROSMsg(mesh->cloud, *cloud);
  return cloud;
}
py::object PolygonMesh_getFaces(const PolygonMesh* mesh) {
  py::list out;
  BOOST_FOREACH(const pcl::Vertices& poly, mesh->polygons) {
    out.append(toInt32_1d((int*)poly.vertices.data(), poly.vertices.size()));
  }
  return out;
}
void PolygonMesh_save(const PolygonMesh* mesh, const std::string& outfile) {
  saveMesh(*mesh, outfile);
}

template<typename T>
void boost_register_cloud_type(const string& pyname) {
  typedef PyCloud<T> PyCloudT;
  typedef PointCloud<T> PointCloudT;
  py::class_<PointCloudT, shared_ptr<PointCloudT> >(pyname.c_str())
//    .def(py::init< shared_ptr<PointCloudT> >() )
    .def("width", &PyCloudT::width)
    .def("height", &PyCloudT::height)
    .def("size", &PyCloudT::size)
    .def("to2dArray", &PyCloudT::to2dArray)
    .def("from2dArray", &PyCloudT::from2dArray)
    .def("save", &PyCloudT::save)
    ;
////    py::implicitly_convertible< shared_ptr< PointCloud<T> >, PyCloudT >();
////    py::implicitly_convertible< shared_ptr< const PointCloud<T> >, PyCloudT >();
//    py::to_python_converter< shared_ptr<PointCloudT>,typename PyCloudT::ToPython>();
//    py::to_python_converter< shared_ptr<const PointCloudT>,typename PyCloudT::ToPython>();
//    typename PyCloudT::FromPython();
  py::implicitly_convertible< boost::shared_ptr<CloudXYZ>, boost::shared_ptr<CloudXYZ const> >();
}



BOOST_PYTHON_MODULE(cloudprocpy) {

  boost_register_cloud_type<PointXYZ>("CloudXYZ");
  boost_register_cloud_type<PointNormal>("CloudXYZN");

  py::def("readPCDXYZ", &readPCD<PointXYZ>);
  py::def("downsampleCloud", &downsampleCloud<PointXYZ>);
  py::def("boxFilter", &boxFilter<PointXYZ>);
  py::def("boxFilterNegative", &boxFilterNegative<PointXYZ>);
  py::def("convexDecomp", &PyConvexDecomp);

  py::class_<pcl::PolygonMesh, shared_ptr<pcl::PolygonMesh> >("PolygonMesh")
      .def("getCloud", &PolygonMesh_getCloud)
      .def("getFaces", &PolygonMesh_getFaces)
      .def("save", &PolygonMesh_save)
      ;

  py::def("meshGP3", &meshGP3);
  py::def("meshOFM", &meshOFM);
  py::def("mlsAddNormals", &mlsAddNormals);

#if 0
  py::class_<CloudGrabber, shared_ptr<CloudGrabber> >("CloudGrabber")
      .def("startXYZ", &CloudGrabber::startXYZ)
      .def("stop", &CloudGrabber::stop)
      .def("getXYZ", &CloudGrabber::getXYZ)
      ;
#endif
}
