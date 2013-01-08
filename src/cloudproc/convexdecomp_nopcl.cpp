#include "convexdecomp_nopcl.hpp"
#include "convexdecomp.hpp"
using namespace pcl;
void ConvexDecomp(float* pointdata, int numpts, int floats_per_point,
    const Eigen::MatrixXf& dirs, float thresh,
    /*optional outputs: */ std::vector<IntVec*> indices, std::vector< IntVec >* hull_indices) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  cloud->reserve(numpts);
  for (int i=0; i < numpts; ++i) {
    float* ppt = pointdata + i*floats_per_point;
    cloud->push_back(PointXYZ(ppt[0], ppt[1], ppt[2]));
  }
  ConvexDecomp(cloud, dirs, thresh, indices, hull_indices);
}

