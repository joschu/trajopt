#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/shared_ptr.hpp>
#include "macros.h"

namespace cloudproc {

class CloudGrabberImpl;

class TRAJOPT_API CloudGrabber {
public:
  CloudGrabber();
  void startXYZ();
  void startXYZRGB();
  void stop();
  pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZ();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getXYZRGB();
private:
  boost::shared_ptr<CloudGrabberImpl> m_impl;
};

}
