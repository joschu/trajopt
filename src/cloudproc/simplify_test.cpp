#include "mesh_simplification.hpp"
#include "cloudproc.hpp"
using namespace cloudproc;
int main() {
  pcl::PolygonMesh::Ptr mesh = loadMesh("/Users/joschu/Downloads/mesh.ply");
  printf("old mesh had %i verts and %i tris\n", mesh->cloud.height*mesh->cloud.width, mesh->polygons.size());
  pcl::PolygonMesh::Ptr simplemesh = quadricSimplifyVTK(*mesh, .1);
  saveMesh(*simplemesh, "/Users/joschu/Downloads/mesh_simple.ply");
  printf("new mesh has %i verts and %i tris\n", simplemesh->cloud.height*simplemesh->cloud.width, simplemesh->polygons.size());

}
