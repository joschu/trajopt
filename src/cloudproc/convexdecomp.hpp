#pragma once
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include "macros.h"

typedef std::vector<int> IntVec;

namespace cloudproc {

TRAJOPT_API void ConvexDecomp1(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, float thresh,
    /*optional outputs: */ std::vector<IntVec>* indices, std::vector< IntVec >* hull_indices);


TRAJOPT_API void ConvexDecomp(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const Eigen::MatrixXf& dirs, float thresh,
    /*optional outputs: */ std::vector<IntVec>* indices, std::vector< IntVec >* hull_indices);
}

