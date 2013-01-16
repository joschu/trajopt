#include "cloudproc.hpp"
#include "convexdecomp.hpp"
using namespace pcl;
using namespace std;
using namespace Eigen;
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "utils/clock.hpp"
#include <boost/format.hpp>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
using namespace cloudproc;
typedef pcl::PointXYZRGBA PointT;
int main(int argc, char** argv) {
  if (argc != 2) {
    printf("usage: %s inputfile.pcd\n", argv[0]);
    abort();
  }

  PointCloud<PointT>::Ptr cloud = readPCD<PointT>(argv[1]);



  pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(20.0f);

  pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
  mps.setMinInliers(10);
  mps.setAngularThreshold(0.017453 * 2.0); //3 degrees
  mps.setDistanceThreshold(0.02); //2cm

   pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
   ne.setInputCloud (cloud);
   ne.compute (*normal_cloud);

   std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions; mps.setInputNormals (normal_cloud);
   mps.setInputCloud (cloud);
   //mps.segmentAndRefine (regions);

   visualization::PCLVisualizer  viewer ("3d Viewer");
   viewer.setBackgroundColor (0,0,0);

   char name[1024];
   unsigned char red [6] = {255,   0,   0, 255, 255,   0};
   unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
   unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};



   pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);
   for (size_t i = 0; i < regions.size (); i++)
   {
     Eigen::Vector3f centroid = regions[i].getCentroid ();
     Eigen::Vector4f model = regions[i].getCoefficients ();
     pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
     pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                        centroid[1] + (0.5f * model[1]),
                                        centroid[2] + (0.5f * model[2]));
     sprintf (name, "normal_%zu", i);
     viewer.addArrow (pt2, pt1, 1.0, 0, 0, false, name);

     contour->points = regions[i].getContour ();
     sprintf (name, "plane_%02zu", i);
     pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i], grn[i], blu[i]);
     viewer.addPointCloud (contour, color, name);
     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
   }
   viewer.spin();


}
