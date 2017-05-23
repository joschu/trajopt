#include <boost/python.hpp>
#include <stdexcept>
#include <boost/python/exception_translator.hpp>
#include <boost/foreach.hpp>
#include "cloudproc/cloudproc.hpp"
#include <iostream>
#include "utils/stl_to_string.hpp"
 #include "cloudgrabber.hpp"
#include "cloudproc/mesh_simplification.hpp"
#include <pcl/point_types.h>
#include <boost/format.hpp>
#include "hacd_interface.hpp"
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
#include <pcl/conversions.h>
#else
#include <pcl/ros/conversions.h>
#endif

using namespace Eigen;
using namespace pcl;
using namespace cloudproc;
using namespace std;
using namespace util;
namespace py = boost::python;
typedef PointCloud<PointXYZ> CloudXYZ;
typedef PointCloud<PointNormal> CloudXYZN;

py::object np_mod, main, globals;

template<typename T>
struct type_traits {
  static const char* npname;
};
template<> const char* type_traits<float>::npname = "float32";
template<> const char* type_traits<int>::npname = "int32";
template<> const char* type_traits<uint8_t>::npname = "uint8";
template<> const char* type_traits<uint16_t>::npname = "uint16";

template <typename T>
T* getPointer(const py::object& arr) {
  long int i = py::extract<long int>(arr.attr("ctypes").attr("data"));
  T* p = (T*)i;
  return p;
}

template<typename T>
py::object toNdarray1(const T* data, size_t dim0) {
  py::object out = np_mod.attr("empty")(py::make_tuple(dim0), type_traits<T>::npname);
  T* p = getPointer<T>(out);
  memcpy(p, data, dim0*sizeof(T));
  return out;
}
template<typename T>
py::object toNdarray2(const T* data, size_t dim0, size_t dim1) {
  py::object out = np_mod.attr("empty")(py::make_tuple(dim0, dim1), type_traits<T>::npname);
  T* pout = getPointer<T>(out);
  memcpy(pout, data, dim0*dim1*sizeof(T));
  return out;
}
template<typename T>
py::object toNdarray3(const T* data, size_t dim0, size_t dim1, size_t dim2) {
  py::object out = np_mod.attr("empty")(py::make_tuple(dim0, dim1, dim2), type_traits<T>::npname);
  T* pout = getPointer<T>(out);
  memcpy(pout, data, dim0*dim1*dim2*sizeof(T));
  return out;
}
template <class T>
struct PyCloud {
  typedef PointCloud<T> PointCloudT;
  static py::object to2dArray(PointCloudT* cloud) {
    return toNdarray2<float>((const float*)cloud->points.data(), cloud->size(), sizeof(T)/sizeof(float));
  }
  static py::object to3dArray(PointCloudT* cloud) {
    return toNdarray3<float>((const float*)cloud->points.data(), cloud->height, cloud->width, sizeof(T)/sizeof(float));
  }
  static void from2dArray(PointCloudT* cloud, py::object arr) {
    arr = np_mod.attr("array")(arr, "float32");
    int npoints = py::extract<int>(arr.attr("shape")[0]);
    int floatfields = py::extract<int>(arr.attr("shape")[1]);
    FAIL_IF_FALSE(floatfields == sizeof(T)/4);
    cloud->resize(npoints);
    float* p = getPointer<float>(arr);
    memcpy(&cloud->points[0], p,  npoints*floatfields*sizeof(float));
  }
  static void from3dArray(PointCloudT* cloud, py::object arr) {
    arr = np_mod.attr("array")(arr, "float32");
    int height = py::extract<int>(arr.attr("shape")[0]);
    int width = py::extract<int>(arr.attr("shape")[1]);
    int floatfields = py::extract<int>(arr.attr("shape")[2]);
    FAIL_IF_FALSE(floatfields==sizeof(T)/4);
    cloud->resize(width*height);
    cloud->height = height;
    cloud->width = width;
    float* p = getPointer<float>(arr);
    memcpy(&cloud->points[0], p,  width*height*floatfields*sizeof(float));
  }
  static void save(PointCloudT* cloud, const std::string& fname) {
    saveCloud(*cloud, fname);
  }
  static int width(PointCloudT* cloud) {return cloud->width;}
  static int height(PointCloudT* cloud){return cloud->height;}
  static int size(PointCloudT* cloud){return cloud->size();}
};

CloudXYZ::Ptr PolygonMesh_getCloud(const PolygonMesh* mesh) {
  CloudXYZ::Ptr cloud(new CloudXYZ());
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::fromPCLPointCloud2(mesh->cloud, *cloud);
#else
  pcl::fromROSMsg(mesh->cloud, *cloud);
#endif
  return cloud;
}
py::object PolygonMesh_getVertices(const PolygonMesh* mesh) {
  py::object cloud(PolygonMesh_getCloud(mesh));

  globals["cloud"] = cloud;
  return py::eval("cloud.to2dArray()[:,:3]", globals);
}
py::object PolygonMesh_getFaces(const PolygonMesh* mesh) {
  py::list out;
  BOOST_FOREACH(const pcl::Vertices& poly, mesh->polygons) {
    out.append(toNdarray1<int>((int*)poly.vertices.data(), poly.vertices.size()));
  }
  return out;
}
py::object PolygonMesh_getTriangles(const PolygonMesh* mesh) {
  vector<int> tridata(mesh->polygons.size()*3);
  for (int i=0; i < mesh->polygons.size(); ++i) {
    const pcl::Vertices& poly = mesh->polygons[i];
    if (poly.vertices.size() != 3) throw std::runtime_error("error: input PolygonMesh contains non-triangle polygons");
    for (int j=0; j < 3; ++j) tridata[3*i+j] = poly.vertices[j];
  }
  return toNdarray2<int>(tridata.data(), mesh->polygons.size(), 3);
}
void PolygonMesh_save(const PolygonMesh* mesh, const std::string& outfile) {
  saveMesh(*mesh, outfile);
}

template<typename T>
void boost_register_cloud_type(const string& pyname) {
  typedef PyCloud<T> PyCloudT;
  typedef PointCloud<T> PointCloudT;
  py::class_<PointCloudT, boost::shared_ptr<PointCloudT> >(pyname.c_str())
    .def("width", &PyCloudT::width)
    .def("height", &PyCloudT::height)
    .def("size", &PyCloudT::size)
    .def("to2dArray", &PyCloudT::to2dArray)
    .def("to3dArray", &PyCloudT::to3dArray)
    .def("from2dArray", &PyCloudT::from2dArray)
    .def("from3dArray", &PyCloudT::from3dArray)
    .def("save", &PyCloudT::save)
    ;
  py::implicitly_convertible< boost::shared_ptr<PointCloudT>, boost::shared_ptr<PointCloudT const> >();
}

py::object pyConvexDecompHACD(const PolygonMesh& mesh, float concavity) {
  vector<PolygonMesh::Ptr> convexmeshes = ConvexDecompHACD(mesh, concavity);
  py::list out;
  BOOST_FOREACH(const PolygonMesh::Ptr& convexmesh, convexmeshes) {
    out.append(convexmesh);
  }
  return out;
}

PointCloud<PointXYZ>::Ptr pyMaskFilter(PointCloud<PointXYZ>::Ptr cloud, py::object arr, bool keep_organized) {
  arr = np_mod.attr("array")(arr, "bool");
  int npoints = py::extract<int>(arr.attr("shape")[0]);
  VectorXb mask(npoints);
  bool* p = getPointer<bool>(arr);
  memcpy(mask.data(), p,  npoints*sizeof(bool));
  return maskFilter<PointXYZ>(cloud, mask, keep_organized);
}

PointCloud<PointXYZ>::Ptr pyComputeConvexHull(PointCloud<PointXYZ>::Ptr cloud, float alpha, int dim) {
  return computeConvexHull(cloud, NULL);
}
PointCloud<PointXYZ>::Ptr pyComputeAlphaShape(PointCloud<PointXYZ>::Ptr cloud, float alpha, int dim) {
  return computeAlphaShape(cloud, alpha, dim, NULL);
}


py::object pyGetNearestNeighborIndices(PointCloud<PointXYZ>::Ptr src, PointCloud<PointXYZ>::Ptr targ) {
  vector<int> inds = getNearestNeighborIndices(src, targ);
  return toNdarray1<int>(inds.data(), inds.size());
}

py::object pyGetRGBD(CloudGrabber* grabber) {
  RGBD::Ptr rgbd = grabber->getRGBD();
  py::object np_rgb = toNdarray3<uint8_t>(rgbd->rgb.data(), 480, 640, 3);
  py::object np_depth = toNdarray2<uint16_t>(rgbd->depth.data(), 480, 640);
  return py::make_tuple(np_rgb, np_depth);
}

BOOST_PYTHON_MODULE(cloudprocpy) {

  np_mod = py::import("numpy");
  main = py::import("__main__");
  globals = main.attr("__dict__");

  boost_register_cloud_type<PointXYZ>("CloudXYZ");
  boost_register_cloud_type<PointXYZRGB>("CloudXYZRGB");
  boost_register_cloud_type<PointNormal>("CloudXYZN");
  boost_register_cloud_type<Normal>("CloudN");

  py::def("readPCDXYZ", &readPCD<PointXYZ>);
  py::def("readPCDXYZRGB", &readPCD<PointXYZRGB>);
  py::def("downsampleCloud", &downsampleCloud<PointXYZ>);
  py::def("boxFilter", &boxFilter<PointXYZ>);
  py::def("boxFilterNegative", &boxFilterNegative<PointXYZ>);
  py::def("medianFilter", &medianFilter<PointXYZ>);
  py::def("fastBilateralFilter", &fastBilateralFilter<PointXYZ>);
  py::def("maskFilter", &pyMaskFilter);

  py::class_<pcl::PolygonMesh, boost::shared_ptr<pcl::PolygonMesh> >("PolygonMesh")
      .def("getCloud", &PolygonMesh_getCloud, "Return underlying point cloud")
      .def("getVertices", &PolygonMesh_getVertices, "Returns V x 3 array of vertices")
      .def("getFaces", &PolygonMesh_getFaces, "Returns a list of lists of indices")
      .def("getTriangles", &PolygonMesh_getTriangles, "Only call on a triangle mesh. Returns F x 3 index array")
      .def("save", &PolygonMesh_save, "Available formats: ply, obj, vtk")
      ;

  py::def("computeConvexHull", &pyComputeConvexHull, (py::arg("cloud")));
  py::def("computeAlphaShape", &pyComputeAlphaShape, (py::arg("cloud"), py::arg("alpha"), py::arg("dim")));
  py::def("getNearestNeighborIndices", &pyGetNearestNeighborIndices, (py::arg("src_cloud"),py::arg("targ_cloud")));
  py::def("meshGP3", &meshGP3, (py::arg("cloud"), py::arg("search_radius")));
  py::def("meshOFM", &meshOFM, (py::arg("cloud"), py::arg("sigma_s")=15, py::arg("sigma_r") = .05));
  py::def("mlsAddNormals", &mlsAddNormals, (py::arg("cloud"), py::arg("search_radius")));
  py::def("integralNormalEstimation", &integralNormalEstimation, (py::arg("cloud"), py::arg("maxDepthChangeFactor")=.02, py::arg("normalSmoothingSize")=10.0));
  py::def("loadMesh", &loadMesh, (py::arg("filename")));
  py::def("quadricSimplifyVTK", &quadricSimplifyVTK, (py::arg("cloud"), py::arg("decimation_frac")));

  py::class_<CloudGrabber, boost::shared_ptr<CloudGrabber> >("CloudGrabber")
      .def("startXYZ", &CloudGrabber::startXYZ, "Start streaming XYZ point clouds")
      .def("getXYZ", &CloudGrabber::getXYZ, "Wait for new XYZ point cloud and return it. If not streaming, will start and stop. (and hang a couple seconds)")
      .def("startXYZRGB", &CloudGrabber::startXYZRGB, "Start streaming XYZRGB")
      .def("getXYZRGB", &CloudGrabber::getXYZRGB, "Wait for new XYZRGB point cloud and return it")
      .def("startRGBD", &CloudGrabber::startRGBD, "Start streaming RGB+Depth images")
      .def("getRGBD", &pyGetRGBD, "Wait for new RGB+Depth and return it")
      .def("stop", &CloudGrabber::stop, "Stop streaming")
      ;

  py::def("convexDecompHACD", &pyConvexDecompHACD, (py::arg("concavityParam") = 100),  "input: mesh. output: list of meshes. concavityParam: see http://kmamou.blogspot.com/2011/10/hacd-hierarchical-approximate-convex.html");

}
