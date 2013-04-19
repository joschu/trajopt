#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/shared_ptr.hpp>
#include "macros.h"

namespace cloudproc {

class CloudGrabberImpl;

struct RGBD {
  typedef boost::shared_ptr<RGBD> Ptr;
  std::vector<unsigned char> rgb;
  std::vector<unsigned short> depth;
  RGBD() : rgb(480*640*3), depth(480*640) {}
};

/**
Simple wrapper around pcl's openni interface, allowing you to query for a point cloud (rather than using a callback)
*/
class TRAJOPT_API CloudGrabber {
public:
  
  CloudGrabber();
  void startXYZ();
  void startXYZRGB();
  void startRGBD();
  void stop();
  pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZ();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getXYZRGB();
  RGBD::Ptr getRGBD();  
private:
  boost::shared_ptr<CloudGrabberImpl> m_impl;
};

}
