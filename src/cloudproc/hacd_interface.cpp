#include "hacd_interface.hpp"
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <hacdHACD.h>
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
#include <pcl/conversions.h>
#else
#include <pcl/ros/conversions.h>
#endif
using HACD::Vec3;
using HACD::Real;
using namespace pcl;
using namespace std;
void polygonMeshFromPointsTriangles(pcl::PolygonMesh& mesh, const vector< Vec3<Real> >& points, const vector< Vec3<long> >& triangles) {
  PointCloud<PointXYZ> cloud;
  cloud.resize(points.size());
  for (int i=0; i < points.size(); ++i) {
    cloud.points[i].x = points[i].X();
    cloud.points[i].y = points[i].Y();
    cloud.points[i].z = points[i].Z();
  }
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
#else
  pcl::toROSMsg(cloud, mesh.cloud);
#endif
  mesh.polygons.resize(triangles.size());
  for (int i=0; i < triangles.size(); ++i) {
    vector<uint32_t>& vertices = mesh.polygons[i].vertices;
    vertices.resize(3);
    vertices[0] = triangles[i].X();
    vertices[1] = triangles[i].Y();
    vertices[2] = triangles[i].Z();
  }
}
void pointsTrianglesFromPolygonMesh(const pcl::PolygonMesh& mesh, vector< Vec3<Real> >& points, vector< Vec3<long> >& triangles) {
  PointCloud<PointXYZ> cloud;
#if PCL_VERSION_COMPARE(>=, 1, 7, 0)
  pcl::fromPCLPointCloud2 (mesh.cloud, cloud);
#else
  pcl::fromROSMsg (mesh.cloud, cloud);
#endif
  points.resize(cloud.size());
  for (int i=0; i < cloud.size(); ++i) {
    points[i].X() = cloud.points[i].x;
    points[i].Y() = cloud.points[i].y;
    points[i].Z() = cloud.points[i].z;
  }
  triangles.resize(mesh.polygons.size());
  for (int i=0; i < triangles.size(); ++i) {
    const vector<uint32_t>& vertices = mesh.polygons[i].vertices;
    triangles[i][0] = vertices[0];
    triangles[i][1] = vertices[1];
    triangles[i][2] = vertices[2];
  }

}

void CallBack(const char * msg, double progress, double concavity, size_t nVertices)
{
    std::cout << msg;
}

vector<pcl::PolygonMesh::Ptr> ConvexDecompHACD(const pcl::PolygonMesh& mesh, float concavity) {

//  std::cout << "Usage: ./testHACD fileName.off minNClusters maxConcavity invertInputFaces addExtraDistPoints addFacesPoints ConnectedComponentsDist targetNTrianglesDecimatedMesh"<< std::endl;
//  std::cout << "Recommended parameters: ./testHACD fileName.off 2 100 0 1 1 30 2000"<< std::endl;

//  size_t nClusters = atoi(argv[2]);
//  double concavity = atof(argv[3]);
//  bool invert = (atoi(argv[4]) == 0)?false:true;
//  bool addExtraDistPoints = (atoi(argv[5]) == 0)?false:true;
//  bool addFacesPoints = (atoi(argv[6]) == 0)?false:true;
//  double ccConnectDist = atof(argv[7]);
//  size_t targetNTrianglesDecimatedMesh = atoi(argv[8]);
//
  int minClusters = 2;
//  bool invert = false;
  bool addExtraDistPoints = true;
  bool addFacesPoints = true;
  float ccConnectDist = 30;
  int targetNTrianglesDecimatedMesh = 3000;

  vector< Vec3<Real> > points;
  vector< Vec3<long> > triangles;
  pointsTrianglesFromPolygonMesh(mesh, points, triangles);

  HACD::HeapManager * heapManager = HACD::createHeapManager(65536*(1000));

  HACD::HACD * const myHACD = HACD::CreateHACD(heapManager);
  myHACD->SetPoints(&points[0]);
  myHACD->SetNPoints(points.size());
  myHACD->SetTriangles(&triangles[0]);
  myHACD->SetNTriangles(triangles.size());
  myHACD->SetCompacityWeight(0.0001);
  myHACD->SetVolumeWeight(0.0);
  myHACD->SetConnectDist(ccConnectDist);               // if two connected components are seperated by distance < ccConnectDist
                                                      // then create a virtual edge between them so the can be merged during
                                                      // the simplification process

  myHACD->SetNClusters(minClusters);                     // minimum number of clusters
  myHACD->SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
  myHACD->SetConcavity(concavity);                     // maximum concavity
  myHACD->SetSmallClusterThreshold(0.25);              // threshold to detect small clusters
  myHACD->SetNTargetTrianglesDecimatedMesh(targetNTrianglesDecimatedMesh); // # triangles in the decimated mesh
  myHACD->SetCallBack(&CallBack);
  myHACD->SetAddExtraDistPoints(addExtraDistPoints);
  myHACD->SetAddFacesPoints(addFacesPoints);

  myHACD->Compute();
  int nClusters = myHACD->GetNClusters();
  vector<PolygonMesh::Ptr> outmeshes(nClusters);
  for (int i=0; i < nClusters; ++i) {
    size_t nPoints = myHACD->GetNPointsCH(i);
    size_t nTriangles = myHACD->GetNTrianglesCH(i);
    vector < Vec3<Real> > hullpoints(nPoints);
    vector < Vec3<long> > hulltriangles(nTriangles);
    myHACD->GetCH(i, &hullpoints[0], &hulltriangles[0]);
    outmeshes[i].reset(new PolygonMesh());
    polygonMeshFromPointsTriangles(*outmeshes[i], hullpoints, hulltriangles);
  }

  HACD::DestroyHACD(myHACD);
  HACD::releaseHeapManager(heapManager);

  return outmeshes;
}
