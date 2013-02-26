#pragma once
#include <pcl/PolygonMesh.h>
#include "macros.h"

namespace cloudproc {


/**
Decimate mesh with vtk function. decimationFrac = fraction of original triangles that new mesh should have.
*/
TRAJOPT_API pcl::PolygonMesh::Ptr quadricSimplifyVTK(pcl::PolygonMesh& in, float decimationFrac);
}

