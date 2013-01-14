#include <boost/python.hpp>
#include <stdexcept>
#include <boost/python/exception_translator.hpp>
#include <boost/foreach.hpp>
#include "cloudproc.hpp"
#include "convexdecomp.hpp"
#include <iostream>
#include "utils/stl_to_string.hpp"
using namespace Eigen;
using namespace pcl;
using namespace cloudproc;
using namespace std;
using namespace util;
namespace py = boost::python;

typedef PointCloud<PointXYZ> PointCloudXYZ;

py::object toInt32_1d(const int* data, size_t sz) {
  py::object np_mod = py::import("numpy");
  py::object out = np_mod.attr("empty")(py::make_tuple(sz), "int32");
  long int i = py::extract<long int>(out.attr("ctypes").attr("data"));
  int* p = (int*)i;
  memcpy(p, data, sz*sizeof(int));
  return out;
}

py::object PointCloudXYZ_xyz(PointCloudXYZ* cloud) {
  py::object np_mod = py::import("numpy");
  py::object out = np_mod.attr("empty")(py::make_tuple(cloud->size(), 3), "float32");
  long int i = py::extract<long int>(out.attr("ctypes").attr("data"));
  float* p = (float*)i;
  BOOST_FOREACH(const PointXYZ& pt, cloud->points) {
    p[0] = pt.x;
    p[1] = pt.y;
    p[2] = pt.z;
    p += 3;
  }
  return out;
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
    .def("xyz", &PointCloudXYZ_xyz)
              ;
  py::def("readPCDXYZ", &readPCDXYZ);
  py::def("downsampleCloud", &downsampleCloud);
  py::def("boxFilter", &boxFilter);
  py::def("convexDecomp", &PyConvexDecomp);


}
