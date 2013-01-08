#pragma once
#include <Eigen/Core>
#include <vector>

typedef std::vector<int> IntVec;

void ConvexDecomp(float* pointdata, int numpts, int floats_per_point, float thresh,
    /*optional outputs: */ std::vector<IntVec*> indices, std::vector< IntVec >* hull_indices);
