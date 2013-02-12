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

typename PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(PointCloud<pcl::PointXYZ>::ConstPtr in, float vsize);

void findConvexHull(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>& out, std::vector<pcl::Vertices>& polygons);

PointCloud<pcl::PointNormal>::Ptr mlsAddNormals(PointCloud<pcl::PointXYZ>::ConstPtr in);

pcl::PolygonMesh::Ptr createMesh_gp3(PointCloud<pcl::PointNormal>::ConstPtr cloud_with_normals);
pcl::PolygonMesh::Ptr createMesh_ofm(PointCloud<pcl::PointXYZ>::ConstPtr cloud);
pcl::PolygonMesh::Ptr createMesh_marchingCubes(PointCloud<pcl::PointXYZ>::ConstPtr cloud);

PointCloud<pcl::PointXYZ>::Ptr toXYZ(PointCloud<pcl::PointNormal>::ConstPtr in);
PointCloud<pcl::PointXYZ>::Ptr boxFilter(PointCloud<pcl::PointXYZ>::ConstPtr, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);
PointCloud<pcl::PointXYZ>::Ptr boxFilterNegative(PointCloud<pcl::PointXYZ>::ConstPtr, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax);


void saveMesh(pcl::PolygonMesh::ConstPtr mesh, const std::string& fname);

}
