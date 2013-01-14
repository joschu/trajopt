#include "cloudproc.hpp"
#include "convexdecomp.hpp"
using namespace pcl;
using namespace cloudproc;


int main (int argc, char** argv)
{
  if (argc != 2) {
    printf("usage: %s inputfile.pcd\n", argv[0]);
    abort();
  }
  PointCloud<PointXYZ>::Ptr cloud = readPCDXYZ(argv[1]);

  PointCloud<PointXYZ>::Ptr cloud_ds = downsampleCloud(cloud, .01f);
  PointCloud<pcl::PointNormal>::Ptr cloudwn = mlsAddNormals(cloud_ds);
  pcl::PolygonMesh::Ptr mesh_gp3 = createMesh_gp3(cloudwn);
  saveMesh(mesh_gp3, "/tmp/mesh_gp3.vtk", VTK);

  pcl::PolygonMesh::Ptr mesh_ofm = createMesh_ofm(cloud);
  saveMesh(mesh_ofm, "/tmp/mesh_ofm.ply", PLY); // can't save to vtk. non-triangles?


//  pcl::io::savePCDFileBinary ("/tmp/pt_normal.pcd",*cloudwn);
//  pcl::io::savePLYFileBinary( "/tmp/pt_normal.ply",*cloudwn);
  return (0);
}
