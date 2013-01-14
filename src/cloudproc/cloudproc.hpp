#pragma once
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <string>

namespace cloudproc {

using pcl::PointCloud;
template <class PointT>
typename PointCloud<PointT>::Ptr readPCD(const std::string& pcdfile);

PointCloud<pcl::PointXYZ>::Ptr readPCDXYZ(const std::string& pcdfile);
//template <class PointT>
//typename PointCloud<PointT>::Ptr downsampleCloud(typename PointCloud<PointT>::Ptr in, float vsize)

typename PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(PointCloud<pcl::PointXYZ>::Ptr in, float vsize);

void findConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>& out, std::vector<pcl::Vertices>& polygons);

PointCloud<pcl::PointNormal>::Ptr mlsAddNormals(PointCloud<pcl::PointXYZ>::Ptr in);

pcl::PolygonMesh::Ptr createMesh_gp3(PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
pcl::PolygonMesh::Ptr createMesh_ofm(PointCloud<pcl::PointXYZ>::Ptr cloud);

PointCloud<pcl::PointXYZ>::Ptr boxFilter(PointCloud<pcl::PointXYZ>::Ptr, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);


enum MeshFormat {
  PLY,
  VTK,
  CUSTOM
};
/**
 * CUSTOM FORMAT
 * 4   [num vertices]
 * x1 y1 z1
 * x2 y2 z2
 * x3 y3 z3
 * x4 y4 z4
 * 2   [num triangles]
 * 0 1 2
 * 3 1 2
 */
void saveMesh(pcl::PolygonMesh::Ptr mesh, const std::string& fname, MeshFormat fmt);

}
