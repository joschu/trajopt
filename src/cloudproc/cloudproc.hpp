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


/////// IO /////////
template <typename T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr readPCD(const std::string& pcdfile);

template <class T>
TRAJOPT_API void saveCloud(const pcl::PointCloud<T>& cloud, const std::string& pcdfile); // instantiate: pcl::PointXYZ pcl::PointXYZRGB pcl::PointNormal pcl::Normal

TRAJOPT_API void saveMesh(const pcl::PolygonMesh& mesh, const std::string& fname);

TRAJOPT_API pcl::PolygonMesh::Ptr loadMesh(const std::string& fname);




/////// Misc processing /////////
template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr downsampleCloud(typename pcl::PointCloud<T>::ConstPtr in, float vsize);

TRAJOPT_API std::vector<int> getNearestNeighborIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr targ);

TRAJOPT_API pcl::PointCloud<pcl::PointXYZ>::Ptr computeConvexHull(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, std::vector<pcl::Vertices>* polygons);
TRAJOPT_API pcl::PointCloud<pcl::PointXYZ>::Ptr computeAlphaShape(pcl::PointCloud<pcl::PointXYZ>::ConstPtr in, float alpha, int dim, std::vector<pcl::Vertices>* polygons);

TRAJOPT_API pcl::PointCloud<pcl::PointNormal>::Ptr mlsAddNormals(PointCloud<pcl::PointXYZ>::ConstPtr in, float searchRadius);

TRAJOPT_API PointCloud<pcl::Normal>::Ptr integralNormalEstimation(PointCloud<pcl::PointXYZ>::ConstPtr in, float maxDepthChangeFactor, float normalSmoothingSize);

template <class T>
TRAJOPT_API typename pcl::PointCloud<pcl::PointXYZ>::Ptr toXYZ(typename pcl::PointCloud<T>::ConstPtr in);


/////// Smoothing /////

template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr medianFilter(typename pcl::PointCloud<T>::ConstPtr in, int windowSize, float maxAllowedMovement); // instantiate: pcl::PointXYZ
/**
sigmaS: standard deviation of the Gaussian used by the bilateral filter for the spatial neighborhood/window. 
PCL default: 15
sigmaR: standard deviation of the Gaussian used to control how much an adjacent pixel is downweighted because of the intensity difference (depth in our case). 
PCL default: .05
*/
template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr fastBilateralFilter(typename pcl::PointCloud<T>::ConstPtr in, float sigmaS, float sigmaR); // instantiate: pcl::PointXYZ


/////// Meshing /////
/**
Greedy projection: see http://pointclouds.org/documentation/tutorials/greedy_projection.php
mu: multiplier of the nearest neighbor distance to obtain the final search radius for each point
typical value: 2.5
maxnn: max nearest neighbors
typical value: 100
*/
TRAJOPT_API pcl::PolygonMesh::Ptr meshGP3(PointCloud<pcl::PointNormal>::ConstPtr cloud, float mu, int maxnn, float searchRadius);

/**
Organized fast mesh: http://docs.pointclouds.org/trunk/classpcl_1_1_organized_fast_mesh.html
edgeLengthPixels: self explanatory
maxEdgeLength: in meters
*/
TRAJOPT_API pcl::PolygonMesh::Ptr meshOFM(PointCloud<pcl::PointXYZ>::ConstPtr cloud, int edgeLengthPixels, float maxEdgeLength);



////// Masking ////

/**
if keep_organized == true, set points where mask=false to nan
if keep_organized == false, return points where mask=true
*/
template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr maskFilter(typename pcl::PointCloud<T>::ConstPtr in, const VectorXb& mask, bool keep_organized);

/**
Return binary mask of points in axis aligned box
*/
template <class T>
TRAJOPT_API VectorXb boxMask(typename pcl::PointCloud<T>::ConstPtr, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax);

/**
Return points in box
*/
template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr boxFilter(typename pcl::PointCloud<T>::ConstPtr in, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax, bool keep_organized) {
  return maskFilter<T>(in, boxMask<T>(in, xmin,ymin,zmin,xmax,ymax,zmax), keep_organized);
}

/**
Return points not in box
*/
template <class T>
TRAJOPT_API typename pcl::PointCloud<T>::Ptr boxFilterNegative(typename pcl::PointCloud<T>::ConstPtr in, float xmin, float ymin, float zmin, float xmax, float ymax, float zmax,  bool keep_organized) {
  return maskFilter<T>(in, 1 - boxMask<T>(in, xmin,ymin,zmin,xmax,ymax,zmax).array(), keep_organized);
}


}
