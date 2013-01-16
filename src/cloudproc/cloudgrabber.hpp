#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>


namespace cloudproc {

class CloudGrabberImpl;

class CloudGrabber {
public:
  CloudGrabber();
  void startXYZ();
  void stop();
  pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZ();
private:
  boost::shared_ptr<CloudGrabberImpl> m_impl;
};

}
