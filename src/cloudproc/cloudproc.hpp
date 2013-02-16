#pragma once
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <string>
#include <Eigen/Core>
#include "macros.h"
namespace cloudproc {

typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

using pcl::PointCloud;

template <typename PointT>
TRAJOPT_API typename pcl::PointCloud<PointT>::Ptr readPCD(const std::string& pcdfile);

template <class PointT>
TRAJOPT_API void saveCloud(const pcl::PointCloud<PointT>& cloud, const std::string& pcdfile);

template <class PointT>
TRAJOPT_API typename pcl::PointCloud<PointT>::Ptr downsampleCloud(typename pcl::PointCloud<PointT>::ConstPtr in, float vsize);

TRAJOPT_API void findConvexHull(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, pcl::PointCloud<pcl::PointXYZ>& out, std::vector<pcl::Vertices>& polygons);

TRAJOPT_API pcl::PointCloud<pcl::PointNormal>::Ptr mlsAddNormals(PointCloud<pcl::PointXYZ>::ConstPtr in, float searchRadius);

TRAJOPT_API pcl::PolygonMesh::Ptr meshGP3(PointCloud<pcl::PointNormal>::ConstPtr cloud, float mu, int maxnn, float searchRadius);
TRAJOPT_API pcl::PolygonMesh::Ptr meshOFM(PointCloud<pcl::PointXYZ>::ConstPtr cloud, int edgeLengthPixels, float maxEdgeLength);

template <class PointT>
TRAJOPT_API typename pcl::PointCloud<pcl::PointXYZ>::Ptr toXYZ(typename pcl::PointCloud<PointT>::ConstPtr in);

template <class PointT>
TRAJOPT_API VectorXb boxMask(typename pcl::PointCloud<PointT>::ConstPtr, const Eigen::Vector3f& mins, const Eigen::Vector3f& maxes);
template <class PointT>
TRAJOPT_API typename pcl::PointCloud<PointT>::Ptr maskFilter(typename pcl::PointCloud<PointT>::ConstPtr in, const VectorXb& mask);

template <class PointT>
TRAJOPT_API typename pcl::PointCloud<PointT>::Ptr boxFilter(typename pcl::PointCloud<PointT>::ConstPtr in, const Eigen::Vector3f& mins, const Eigen::Vector3f& maxes) {
  return maskFilter<PointT>(in, boxMask<PointT>(in, mins, maxes));
}
template <class PointT>
TRAJOPT_API typename pcl::PointCloud<PointT>::Ptr boxFilterNegative(typename pcl::PointCloud<PointT>::ConstPtr in, const Eigen::Vector3f& mins, const Eigen::Vector3f& maxes) {
  return maskFilter<PointT>(in, 1 - boxMask<PointT>(in, mins, maxes).array());
}


TRAJOPT_API void saveMesh(const pcl::PolygonMesh& mesh, const std::string& fname);


}
