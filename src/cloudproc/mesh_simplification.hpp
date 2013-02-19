#pragma once
#include <pcl/PolygonMesh.h>
#include "macros.h"

namespace cloudproc {
TRAJOPT_API pcl::PolygonMesh::Ptr quadricSimplifyVTK(pcl::PolygonMesh& in, float decimationFrac);
}

