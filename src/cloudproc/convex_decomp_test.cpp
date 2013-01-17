#include "cloudproc.hpp"
#include "convexdecomp.hpp"
using namespace pcl;
using namespace std;
using namespace Eigen;
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include "utils/clock.hpp"
#include <boost/format.hpp>
#include "sphere_sampling.hpp"
using namespace util;
using namespace cloudproc;

namespace cloudproc {
extern void ConvexDecomp(const pcl::PointCloud<pcl::PointNormal>::ConstPtr& cloud, float thresh,
    /*optional outputs: */ std::vector<IntVec>* indices, std::vector< IntVec >* hull_indices);
}
int main (int argc, char** argv)
{
  if (argc != 2) {
    printf("usage: %s inputfile.pcd\n", argv[0]);
    abort();
  }

#if 1
  PointCloud<PointXYZ>::Ptr cloud = readPCDXYZ(argv[1]);
  cloud=downsampleCloud(cloud, .005);
  cloud=boxFilter(cloud, -10,10,-10,10,-10,10);

#else
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  cloud->push_back(PointXYZ(0,0,0));
  cloud->push_back(PointXYZ(0,1,0));
  cloud->push_back(PointXYZ(1,0,0));
  cloud->push_back(PointXYZ(1,1,0));
  cloud->push_back(PointXYZ(.5,.5,0));




  RowMat dirs(4,3);
  dirs << 1,0,0,
      -1,0,0,
      0,1,0,
      0,-1,0;
#endif

  vector<vector<int> > label2inds;
  double tStart = GetClock();

  PointCloud<PointNormal>::Ptr cloud_wn = mlsAddNormals(cloud);
  ConvexDecomp(cloud_wn, .1, &label2inds, NULL);


  cout << "time: " << GetClock() - tStart << endl;

  typedef pair<int,int> IntPair;
  vector< IntPair > size_label;
  for (int i=0; i < label2inds.size(); ++i) {
    size_label.push_back(IntPair(-label2inds[i].size(), i));
  }

  std::sort(size_label.begin(), size_label.end());


  visualization::PCLVisualizer  viewer ("3d Viewer");
  viewer.setBackgroundColor (0,0,0);


  PointCloud<Normal> normals;
  PointCloud<PointXYZ> points;
  BOOST_FOREACH(const PointNormal& pn, cloud_wn->points) {
    points.push_back(PointXYZ(pn.x, pn.y, pn.z));
    normals.push_back(Normal(pn.normal_x, pn.normal_y, pn.normal_z));
    cout << normals.points.back().getNormalVector3fMap().norm() << endl;
  }
  viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(points.makeShared(), normals.makeShared(),1);

#if 1
  for (int i=0; i < size_label.size(); ++i) {
    int label = size_label[i].second;
    cout << label << " " << label2inds[label].size() << endl;
    pcl::PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);



    float r=rand()%255, g = rand()%255, b = rand()%255;

    for (int j=0; j < label2inds[label].size(); ++j) {
      PointNormal  oldpt = cloud_wn->points[label2inds[label][j]];
      PointXYZRGB newpt;
      newpt.x = oldpt.x;
      newpt.y = oldpt.y;
      newpt.z = oldpt.z;
      newpt.r = r;
      newpt.g = g;
      newpt.b = b;
      cloud->push_back(newpt);
    }
    viewer.addPointCloud(cloud, (boost::format("cloud_%i")%i).str());
    viewer.spin();
  }
#else

  for (int i=0; i < label2inds.size(); ++i) {
    pcl::PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

    float r=rand()%255, g = rand()%255, b = rand()%255;

    vector<int> inds = label2inds[i];
    for (int j=0; j < inds.size(); ++j) {
      PointNormal  oldpt = cloud_wn->points[inds[j]];
      PointXYZRGB newpt;
      newpt.x = oldpt.x;
      newpt.y = oldpt.y;
      newpt.z = oldpt.z;
      newpt.r = r;
      newpt.g = g;
      newpt.b = b;
      cloud->push_back(newpt);
    }
    viewer.addPointCloud(cloud, (boost::format("cloud_%i")%i).str());
    viewer.spin();
  }
#endif
  return (0);
}
