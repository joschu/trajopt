#include <boost/python.hpp>
#include <stdexcept>
#include <boost/python/exception_translator.hpp>
#include <boost/foreach.hpp>
#include "cloudproc.hpp"
#include "convexdecomp.hpp"
#include <iostream>
#include "utils/stl_to_string.hpp"
#include "cloudgrabber.hpp"
using namespace Eigen;
using namespace pcl;
using namespace cloudproc;
using namespace std;
using namespace util;
namespace py = boost::python;

typedef PointCloud<PointXYZ> PointCloudXYZ;


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

py::object PointCloudXYZ_to2dArray(const PointCloudXYZ* cloud) {
  py::object np_mod = py::import("numpy");
  py::object out = np_mod.attr("empty")(py::make_tuple(cloud->size(), 3), "float32");
  float* p = getPointer<float>(out);
  BOOST_FOREACH(const PointXYZ& pt, cloud->points) {
    p[0] = pt.x;
    p[1] = pt.y;
    p[2] = pt.z;
    p += 3;
  }
  return out;
}


void PointCloudXYZ_from2dArray(PointCloudXYZ* cloud, py::object arr) {
  int rows = py::extract<int>(arr.attr("shape")[0]);
  int cols = py::extract<int>(arr.attr("shape")[1]);
  if (cols != 3) throw std::runtime_error("array must have 3 columns");
  cloud->resize(rows);
  string dtype = py::extract<string>(arr.attr("dtype").attr("name"));
  if (dtype != string("float32")) throw std::runtime_error("array must have dtype float32");
  float* p = getPointer<float>(arr);
  BOOST_FOREACH(PointXYZ& pt, cloud->points) {
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    p += 3;
  }
}

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

BOOST_PYTHON_MODULE(cloudprocpy) {

  py::class_<PointCloudXYZ, boost::shared_ptr<PointCloudXYZ> >("PointCloudXYZ")
    .def_readonly("width", &PointCloudXYZ::width)
    .def_readonly("height", &PointCloudXYZ::height)
    .def("size", &PointCloudXYZ::size)
    .def("to2dArray", &PointCloudXYZ_to2dArray)
    .def("from2dArray", &PointCloudXYZ_from2dArray)
              ;
  py::def("readPCDXYZ", &readPCDXYZ);
  py::def("downsampleCloud", &downsampleCloud);
  py::def("boxFilter", &boxFilter);
  py::def("boxFilterNegative", &boxFilterNegative);
  py::def("convexDecomp", &PyConvexDecomp);


  py::implicitly_convertible< boost::shared_ptr<PointCloudXYZ>, boost::shared_ptr<PointCloudXYZ const> >();

#if 0
  py::class_<CloudGrabber, boost::shared_ptr<CloudGrabber> >("CloudGrabber")
      .def("startXYZ", &CloudGrabber::startXYZ)
      .def("stop", &CloudGrabber::stop)
      .def("getXYZ", &CloudGrabber::getXYZ)
      ;
#endif
}
