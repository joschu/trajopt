#include "cloudproc.hpp"
#include <pcl/io/pcd_io.h>
#include "boost/format.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/convex_hull.h>
using namespace std;
using namespace pcl;

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


template <class PointT>
typename PointCloud<PointT>::Ptr readPCD(const std::string& pcdfile) {
  typename PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>);
  if (pcl::io::loadPCDFile<PointT> (pcdfile, *cloud) == -1) {
    FILE_OPEN_ERROR(pcdfile);
    }
  return cloud;
}
template PointCloud<PointXYZRGB>::Ptr readPCD<PointXYZRGB>(const std::string& pcdfile);
template PointCloud<PointXYZRGBA>::Ptr readPCD<PointXYZRGBA>(const std::string& pcdfile);
template PointCloud<PointXYZ>::Ptr readPCD<PointXYZ>(const std::string& pcdfile);

PointCloud<pcl::PointXYZ>::Ptr readPCDXYZ(const std::string& pcdfile) {
  sensor_msgs::PointCloud2 cloud_blob;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile (pcdfile, cloud_blob) != 0) FILE_OPEN_ERROR(pcdfile);
  pcl::fromROSMsg (cloud_blob, *cloud);
  return cloud;
}


///////////////////////////

#define INSTANTIATE_downsampleCloud(PointT)     \
PointCloud<PointT>::Ptr downsampleCloud(PointCloud<PointT>::Ptr in, float vsize) {\
  PointCloud<PointT>::Ptr out (new PointCloud<PointT>);    \
  pcl::VoxelGrid< PointT > sor;                 \
  sor.setInputCloud (in);\
  sor.setLeafSize (vsize, vsize, vsize);\
  sor.filter (*out);\
  return out;\
}

INSTANTIATE_downsampleCloud(PointXYZ)
INSTANTIATE_downsampleCloud(PointXYZRGB)

//////////////////////////


void findConvexHull(PointCloud<pcl::PointXYZ>::Ptr in, PointCloud<pcl::PointXYZ>& out, std::vector<Vertices>& polygons) {
  pcl::ConvexHull<PointXYZ> chull;
  chull.setInputCloud (in);
  chull.reconstruct (out, polygons);
}

PointCloud<pcl::PointNormal>::Ptr mlsAddNormals(PointCloud<pcl::PointXYZ>::Ptr in) {
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (in);

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  mls.setInputCloud (in);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  mls.process (*cloud_with_normals);
  return cloud_with_normals;
}




pcl::PolygonMesh::Ptr createMesh_gp3(PointCloud<pcl::PointNormal>::Ptr cloud_with_normals) {
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(0.025);

  // Set typical values for the parameters
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
  gp3.setMinimumAngle(M_PI / 18); // 10 degrees
  gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
  gp3.setNormalConsistency(false);

  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(*triangles);
  return triangles;
}


pcl::PolygonMesh::Ptr createMesh_ofm(PointCloud<pcl::PointXYZ>::Ptr cloud) {
  pcl::OrganizedFastMesh<pcl::PointXYZ> ofm;
  ofm.setInputCloud(cloud);
  ofm.setTrianglePixelSize (3);
  ofm.setTriangulationType (pcl::OrganizedFastMesh<PointXYZ>::QUAD_MESH);
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  ofm.reconstruct(mesh->polygons);
  pcl::toROSMsg(*cloud, mesh->cloud);
  mesh->header = cloud->header;
  return mesh;
}


void saveTrimeshCustomFmt(pcl::PolygonMesh::Ptr mesh, const std::string& fname) {
  ofstream o(fname.c_str());
  if (o.bad()) FILE_OPEN_ERROR(fname);
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
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

PointCloud<pcl::PointXYZ>::Ptr boxFilter(PointCloud<pcl::PointXYZ>::Ptr in, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax) {
  PointCloud<pcl::PointXYZ>::Ptr out(new PointCloud<pcl::PointXYZ>());
  BOOST_FOREACH(const PointXYZ& pt, in->points) {
    if (pt.x >= xmin && pt.x <= xmax && pt.y >= ymin && pt.y <= ymax && pt.z >= zmin && pt.z <= zmax) {
      out->points.push_back(pt);
    }
  }
  setWidthToSize(out);
  return out;
}

PointCloud<pcl::PointXYZ>::Ptr boxFilterNegative(PointCloud<pcl::PointXYZ>::Ptr in, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax) {
  PointCloud<pcl::PointXYZ>::Ptr out(new PointCloud<pcl::PointXYZ>());
  BOOST_FOREACH(const PointXYZ& pt, in->points) {
    if (!((pt.x >= xmin && pt.x <= xmax && pt.y >= ymin && pt.y <= ymax && pt.z >= zmin && pt.z <= zmax))) {
      out->points.push_back(pt);
    }
  }
  setWidthToSize(out);
  return out;
}

void saveMesh(pcl::PolygonMesh::Ptr mesh, const std::string& fname, MeshFormat fmt) {


  switch (fmt) {
  case PLY:
    pcl::io::savePLYFile (fname, *mesh);
    break;
  case VTK:
    pcl::io::saveVTKFile (fname, *mesh);
    break;
  case CUSTOM:
    saveTrimeshCustomFmt(mesh, fname);
    break;
  default:
    throw runtime_error( (boost::format("invalid mesh format %i")%fmt).str() );
  }
}

}
