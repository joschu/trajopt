#include "cloudproc.hpp"
#include <pcl/io/pcd_io.h>
#include "boost/format.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include "pcl/io/vtk_lib_io.h"
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include "pcl/impl/instantiate.hpp"
#include <boost/filesystem.hpp>
#include <pcl/features/integral_image_normal.h>
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
#include <pcl/filters/median_filter.h>
#include <pcl/filters/fast_bilateral.h>
#endif

using namespace std;
using namespace pcl;
namespace fs = boost::filesystem;

#define FILE_OPEN_ERROR(fname) throw runtime_error( (boost::format("couldn't open %s")%fname).str() )

namespace cloudproc {

template <class CloudT>
int cloudSize(const CloudT& cloud) {
  return cloud.width * cloud.height;
}

template <class CloudT>
void setWidthToSize(const CloudT& cloud) {
  cloud->width = cloud->points.size();
  cloud->height = 1;
}

template <class T>
typename pcl::PointCloud<T>::Ptr readPCD(const std::string& pcdfile) {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::PCLPointCloud2 cloud_blob;
#else
  sensor_msgs::PointCloud2 cloud_blob;
#endif
  typename pcl::PointCloud<T>::Ptr cloud (new typename pcl::PointCloud<T>);
  if (pcl::io::loadPCDFile (pcdfile, cloud_blob) != 0) FILE_OPEN_ERROR(pcdfile);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
#else
  pcl::fromROSMsg (cloud_blob, *cloud);
#endif
  return cloud;
}

template<class T>
void saveCloud(const typename pcl::PointCloud<T>& cloud, const std::string& fname) {
  std::string ext = fs::extension(fname);
  if (ext == ".pcd")   pcl::io::savePCDFileBinary(fname, cloud);
  else if (ext == ".ply") PRINT_AND_THROW("not implemented");//pcl::io::savePLYFile(fname, cloud, true);
  else throw std::runtime_error( (boost::format("%s has unrecognized extension")%fname).str() );
}

///////////////////////////

template<class T>
typename pcl::PointCloud<T>::Ptr downsampleCloud(typename pcl::PointCloud<T>::ConstPtr in, float vsize) {
  typename pcl::PointCloud<T>::Ptr out (new typename pcl::PointCloud<T>);
  pcl::VoxelGrid< T > sor;
  sor.setInputCloud (in);
  sor.setLeafSize (vsize, vsize, vsize);
  sor.filter (*out);
  return out;
}

vector<int> getNearestNeighborIndices(PointCloud<PointXYZ>::Ptr src, PointCloud<PointXYZ>::Ptr targ) {
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>(true));
  tree->setEpsilon(0);
  tree->setInputCloud (targ);
  vector<int> out;
  out.reserve(src->size());

  vector<int> neighb_inds(1);
  vector<float> sqdists(1);
  int k_neighbs=1;
  BOOST_FOREACH(const PointXYZ& pt, src->points) {
     int n_neighbs = tree->nearestKSearch(pt, k_neighbs, neighb_inds, sqdists);
     FAIL_IF_FALSE(n_neighbs > 0);
     out.push_back(neighb_inds[0]);    
  }
  return out;
}

//////////////////////////




PointCloud<pcl::PointXYZ>::Ptr computeConvexHull(PointCloud<pcl::PointXYZ>::ConstPtr in, std::vector<Vertices>* polygons) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new PointCloud<PointXYZ>()); 
  pcl::ConvexHull<PointXYZ> chull;
  chull.setInputCloud (in);
  if (polygons != NULL) chull.reconstruct (*out, *polygons);
  else chull.reconstruct(*out);
  return out;
}
PointCloud<pcl::PointXYZ>::Ptr computeAlphaShape(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, float alpha, int dim, std::vector<pcl::Vertices>* polygons) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new PointCloud<PointXYZ>()); 
  pcl::ConcaveHull<PointXYZ> chull;
  chull.setInputCloud (in);
  chull.setDimension(dim);
  if (polygons != NULL) chull.reconstruct (*out, *polygons);
  else chull.reconstruct(*out);
  return out;
}


PointCloud<pcl::PointNormal>::Ptr mlsAddNormals(PointCloud<pcl::PointXYZ>::ConstPtr in, float searchRadius) {
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (in);

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  mls.setInputCloud (in);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.04);
  mls.process (*cloud_with_normals);
  return cloud_with_normals;
}


PointCloud<pcl::Normal>::Ptr integralNormalEstimation(PointCloud<pcl::PointXYZ>::ConstPtr in, float maxDepthChangeFactor, float normalSmoothingSize) {
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(maxDepthChangeFactor);
  ne.setNormalSmoothingSize(normalSmoothingSize);
  
  ne.setInputCloud(in);
  ne.compute(*normals);
  return normals;
}
#if 0
pcl::PolygonMesh::Ptr createMesh_MarchingCubes(PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals) {
  pcl::PolygonMesh::Ptr triangles(new PolygonMesh());
    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    pcl::MarchingCubesGreedy<pcl::PointNormal> mcg;
    mcg.setInputCloud(cloud_with_normals); 
    mcg.setSearchMethod(tree2); 
    mcg.setIsoLevel(.1);
    mcg.setGridResolution(100,100,100);
    mcg.reconstruct (*triangles);

    return triangles;
}
#endif

pcl::PolygonMesh::Ptr meshGP3(PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals, float mu, int maxnn, float searchRadius) {
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(searchRadius);

  // Set typical values for the parameters
  gp3.setMu(mu);
  gp3.setMaximumNearestNeighbors(maxnn);
  gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
  gp3.setMinimumAngle(M_PI / 18); // 10 degrees
  gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
  gp3.setNormalConsistency(false);

  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(*triangles);
  return triangles;
}


pcl::PolygonMesh::Ptr meshOFM(PointCloud<pcl::PointXYZ>::ConstPtr cloud, int edgeLengthPixels, float maxEdgeLength) {
  pcl::OrganizedFastMesh<pcl::PointXYZ> ofm;
  ofm.setInputCloud(cloud);
  ofm.setTrianglePixelSize (edgeLengthPixels);
  ofm.setMaxEdgeLength(maxEdgeLength);
  ofm.setTriangulationType (pcl::OrganizedFastMesh<PointXYZ>::TRIANGLE_ADAPTIVE_CUT);
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  ofm.reconstruct(mesh->polygons);
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::toPCLPointCloud2(*cloud, mesh->cloud);
#else
  pcl::toROSMsg(*cloud, mesh->cloud);
#endif
  mesh->header = cloud->header;
  return mesh;
}

#if 0
pcl::PolygonMesh::Ptr createMesh(PointCloud<pcl::PointNormal>::ConstPtr cloud, MeshMethod method) {
  switch (method) {
  case MarchingCubes:
    return createMesh_MarchingCubes(cloud);
  case GP3:
    return createMesh_GP3(cloud);
  default:
    throw std::runtime_error("invalid meshing method");
  }
}
#endif

#if 0
void saveTrimeshCustomFmt(pcl::PolygonMesh::ConstPtr mesh, const std::string& fname) {
  ofstream o(fname.c_str());
  if (o.bad()) FILE_OPEN_ERROR(fname);
  pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>());
  pcl::fromROSMsg(mesh->cloud, *cloud);
  o << cloud->size() << endl;
  BOOST_FOREACH(const PointXYZ& pt, cloud->points) {
    o << pt.x << " " << pt.y << " " << pt.z;
  }
  o << mesh->polygons.size() << endl;
  BOOST_FOREACH(const pcl::Vertices& verts, mesh->polygons) {
    if (verts.vertices.size() != 3) throw runtime_error("only triangles are supported");
    o << verts.vertices[0] << " " << verts.vertices[1] << " " << verts.vertices[2] << endl;
  }
}
#endif

template <typename T>
pcl::PointCloud<pcl::PointXYZ>::Ptr toXYZ(typename pcl::PointCloud<T>::ConstPtr in) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<PointXYZ>());
  out->reserve(in->size());
  out->width = in->width;
  out->height = in->height;
  BOOST_FOREACH(const T& pt, in->points) {
    out->points.push_back(PointXYZ(pt.x, pt.y, pt.z));
  }
  return out;
}

template <class T>
VectorXb boxMask(typename pcl::PointCloud<T>::ConstPtr in, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax) {
  int i=0;
  VectorXb out(in->size());
  BOOST_FOREACH(const T& pt, in->points) {
    out[i] = (pt.x >= xmin && pt.x <= xmax && pt.y >= ymin && pt.y <= ymax && pt.z >= zmin && pt.z <= zmax);
    ++i;
  }
  return out;
}

template <class T>
typename pcl::PointCloud<T>::Ptr maskFilterDisorganized(typename pcl::PointCloud<T>::ConstPtr in, const VectorXb& mask) {
  int n = mask.sum();
  typename pcl::PointCloud<T>::Ptr out(new typename pcl::PointCloud<T>());
  out->points.reserve(n);
  for (int i=0; i < mask.size(); ++i) {
    if (mask[i]) out->points.push_back(in->points[i]);
  }
  setWidthToSize(out);
  return out;
}

template <class T>
typename pcl::PointCloud<T>::Ptr maskFilterOrganized(typename pcl::PointCloud<T>::ConstPtr in, const VectorXb& mask) {
  typename pcl::PointCloud<T>::Ptr out(new typename pcl::PointCloud<T>(*in));
  for (int i=0; i < mask.size(); ++i) {
    if (!mask[i]) {
      T& pt = out->points[i];
      pt.x = NAN;
      pt.y = NAN;
      pt.z = NAN;
    }
  }
  return out;
}

template <class T>
typename pcl::PointCloud<T>::Ptr maskFilter(typename pcl::PointCloud<T>::ConstPtr in, const VectorXb& mask, bool keep_organized) {
  if (keep_organized) return maskFilterOrganized<T>(in, mask);
  else return maskFilterDisorganized<T>(in, mask);
}

template <class T>
typename pcl::PointCloud<T>::Ptr medianFilter(typename pcl::PointCloud<T>::ConstPtr in, int windowSize, float maxAllowedMovement) {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::MedianFilter<T> mf;
  mf.setWindowSize(windowSize);
  mf.setMaxAllowedMovement(maxAllowedMovement);
  typename PointCloud<T>::Ptr out(new PointCloud<T>());
  mf.setInputCloud(in);
  mf.filter(*out);
  return out;
#else 
  PRINT_AND_THROW("not implemented");
#endif
}

template <class T>
typename pcl::PointCloud<T>::Ptr fastBilateralFilter(typename pcl::PointCloud<T>::ConstPtr in, float sigmaS, float sigmaR) {
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::FastBilateralFilter<T> mf;
  mf.setSigmaS(sigmaS);
  mf.setSigmaR(sigmaR);
  typename PointCloud<T>::Ptr out(new PointCloud<T>());
  mf.setInputCloud(in);
  mf.applyFilter(*out);
  return out;
#else
  PRINT_AND_THROW("not implemented");
#endif
}

#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
void removenans(pcl::PCLPointCloud2& cloud, float fillval=0);
void removenans(pcl::PCLPointCloud2& cloud, float fillval) {
#else
void removenans(sensor_msgs::PointCloud2& cloud, float fillval=0);
void removenans(sensor_msgs::PointCloud2& cloud, float fillval) {
#endif
  int npts = cloud.width * cloud.height;
  for (int i=0; i < npts; ++i) {
    float* ptdata = (float*)(cloud.data.data() + cloud.point_step * i);
    for (int j=0; j < 3; ++j) {
      if (!isfinite(ptdata[j])) ptdata[j] = fillval;
    }
  }
}

void saveMesh(const pcl::PolygonMesh& origmesh, const std::string& fname) {
  pcl::PolygonMesh mesh = origmesh;
  removenans(mesh.cloud);

  string ext = fs::extension(fname);
  int errcode;
  if (ext == ".ply") {
//    errcode = pcl::io::savePLYFileBinary (fname, mesh);
    PRINT_AND_THROW("not implemented");
  }
  else if (ext == ".obj") {
    errcode = pcl::io::saveOBJFile (fname, mesh);
  }
  else if (ext == ".vtk") {
    errcode = pcl::io::saveVTKFile (fname, mesh);
  }
  else PRINT_AND_THROW(boost::format("filename %s had unrecognized extension")%fname);
  if (errcode) PRINT_AND_THROW(boost::format("saving mesh to file %s failed")%fname);
}

pcl::PolygonMesh::Ptr loadMesh(const std::string& fname) {
  pcl::PolygonMesh::Ptr out(new PolygonMesh());

  string ext = fs::extension(fname);

  if (ext == ".ply") {
    pcl::io::loadPolygonFilePLY(fname, *out);
  }
  else if (ext == ".obj") {
    pcl::io::loadPolygonFileOBJ (fname, *out);
  }
  else if (ext == ".vtk") {
    pcl::io::loadPolygonFileVTK (fname, *out);
  }
  else PRINT_AND_THROW(boost::format("filename %s had unrecognized extension")%fname);
  return out;
}

#include "autogen_instantiations.cpp"
}

