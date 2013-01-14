#pragma once
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>

typedef std::vector<int> IntVec;

void ConvexDecomp1(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, float thresh,
    /*optional outputs: */ std::vector<IntVec>* indices, std::vector< IntVec >* hull_indices);


void ConvexDecomp(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, const Eigen::MatrixXf& dirs, float thresh,
    /*optional outputs: */ std::vector<IntVec>* indices, std::vector< IntVec >* hull_indices);
