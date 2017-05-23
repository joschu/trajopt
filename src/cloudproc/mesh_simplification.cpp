#include "mesh_simplification.hpp"
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkPoints.h>
#include <vtkPolyLine.h>
#include <vtkPolyData.h>
#include "cloudproc/vtkQuadricDecimation2.h"
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "macros.h"
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
#include <pcl/conversions.h>
#else
#include <pcl/ros/conversions.h>
#endif

using namespace pcl;
namespace {
void toPCL(vtkPolyData& in, pcl::PolygonMesh& out) {
  int nPts = in.GetNumberOfPoints();
//  vtkPoints* points = in.GetPoints();
  pcl::PointCloud<PointXYZ> cloud;
  cloud.resize(nPts);
  for (int i = 0; i < nPts; ++i) {
    double pointdata[3];
    in.GetPoint(i, pointdata);
    cloud.points[i].x = pointdata[0];
    cloud.points[i].y = pointdata[1];
    cloud.points[i].z = pointdata[2];
  }
  int nPolygons = in.GetNumberOfCells();
  out.polygons.resize(nPolygons);
  for (int i = 0; i < nPolygons; ++i) {
//    vtkIdType idxdata[3];
//    unsigned short nPolyPts;
//    in.GetPointCells(i, nPolyPts, idxdata);

    vtkIdType numIds; // to hold the size of the cell
    vtkIdType *idxdata; // to hold the ids in the cell
    in.GetCellPoints(i, numIds, idxdata);

    std::vector<uint32_t>& vertices = out.polygons[i].vertices;
    vertices.resize(3);
    vertices[0] = idxdata[0];
    vertices[1] = idxdata[1];
    vertices[2] = idxdata[2];
  }
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::toPCLPointCloud2(cloud, out.cloud);
#else
  pcl::toROSMsg(cloud, out.cloud);
#endif

}
void toVTK(pcl::PolygonMesh& in, vtkPolyData& out) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::fromPCLPointCloud2(in.cloud, cloud);
#else
  pcl::fromROSMsg(in.cloud, cloud);
#endif

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  BOOST_FOREACH(const PointXYZ& pt, cloud.points) {
    points->InsertNextPoint((float*) &pt);
  }
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  BOOST_FOREACH(const pcl::Vertices& verts, in.polygons) {
    assert(verts.vertices.size() == 3);
    long long inds[3] = { verts.vertices[0], verts.vertices[1],
        verts.vertices[2] };
    cells->InsertNextCell(3, inds);
  }
  out.SetPoints(points);
  out.SetPolys(cells);
}
}

namespace cloudproc {
pcl::PolygonMesh::Ptr quadricSimplifyVTK(pcl::PolygonMesh& in, float meshDecimationFrac) {
  if (meshDecimationFrac < 0 || meshDecimationFrac > 1) PRINT_AND_THROW("expected 0 <= meshDecimationFrac <= 1");

  vtkSmartPointer<vtkPolyData> inputPolyData =
      vtkSmartPointer<vtkPolyData>::New();
  toVTK(in, *inputPolyData);

  vtkSmartPointer<vtkQuadricDecimation2> decimate = vtkSmartPointer<
      vtkQuadricDecimation2>::New();
  decimate->SetTargetReduction(1-meshDecimationFrac);
  decimate->SetInput(inputPolyData.GetPointer());
  decimate->Update();

  vtkSmartPointer<vtkPolyData> decimated = vtkSmartPointer<vtkPolyData>::New();
  decimated->ShallowCopy(decimate->GetOutput());

  PolygonMesh::Ptr out(new PolygonMesh());
  toPCL(*decimated, *out);
  return out;
}
}
