#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <string>
#include <Eigen/Core>
#include "macros.h"
namespace cloudproc {

typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

using pcl::PointCloud;

template <typename T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr readPCD(const std::string& pcdfile);

template <class T>
TRAJOPT_API void saveCloud(const pcl::PointCloud<T>& cloud, const std::string& pcdfile);

template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr downsampleCloud(typename pcl::PointCloud<T>::ConstPtr in, float vsize);

TRAJOPT_API void findConvexHull(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>& out, std::vector<pcl::Vertices>& polygons);

TRAJOPT_API pcl::PointCloud<pcl::PointNormal>::Ptr mlsAddNormals(PointCloud<pcl::PointXYZ>::ConstPtr in, float searchRadius);

TRAJOPT_API pcl::PolygonMesh::Ptr meshGP3(PointCloud<pcl::PointNormal>::ConstPtr cloud, float mu, int maxnn, float searchRadius);
TRAJOPT_API pcl::PolygonMesh::Ptr meshOFM(PointCloud<pcl::PointXYZ>::ConstPtr cloud, int edgeLengthPixels, float maxEdgeLength);

template <class T>
TRAJOPT_API typename pcl::PointCloud<pcl::PointXYZ>::Ptr toXYZ(typename pcl::PointCloud<T>::ConstPtr in);

template <class T>
TRAJOPT_API VectorXb boxMask(typename pcl::PointCloud<T>::ConstPtr, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax);
template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr maskFilter(typename pcl::PointCloud<T>::ConstPtr in, const VectorXb& mask);

template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr boxFilter(typename pcl::PointCloud<T>::ConstPtr in, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax) {
  return maskFilter<T>(in, boxMask<T>(in, xmin,ymin,zmin,xmax,ymax,zmax));
}
template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr boxFilterNegative(typename pcl::PointCloud<T>::ConstPtr in, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax) {
  return maskFilter<T>(in, 1 - boxMask<T>(in, xmin,ymin,zmin,xmax,ymax,zmax).array());
}

template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr medianFilter(typename pcl::PointCloud<T>::ConstPtr in, int windowSize, float maxAllowedMovement); // instantiate: pcl::PointXYZ

template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr fastBilateralFilter(typename pcl::PointCloud<T>::ConstPtr in, float sigmaS, float sigmaR); // instantiate: pcl::PointXYZ

TRAJOPT_API void saveMesh(const pcl::PolygonMesh& mesh, const std::string& fname);
TRAJOPT_API pcl::PolygonMesh::Ptr loadMesh(const std::string& fname);

}
