#include <pcl/io/openni_grabber.h>
#include "cloudgrabber.hpp"
#include <boost/function.hpp>
#include <cstdio>
using namespace pcl;

namespace cloudproc {

class CloudGrabberImpl{
public:
  bool m_xyz_requested;
  PointCloud<PointXYZ>::Ptr m_xyz;
  pcl::OpenNIGrabber m_interface;
  boost::signals2::connection c;

  CloudGrabberImpl() : m_xyz_requested(false) {}

  void xyzrgba_cb (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
    printf("callback!\n");
  }
  void xyz_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
    if (m_xyz_requested) {
      m_xyz.reset(new PointCloud<PointXYZ>(*cloud));
      m_xyz_requested = false;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZ() {
    m_xyz.reset();
    m_xyz_requested = true;
    while (m_xyz_requested) sleep(.01);
    return m_xyz;
  }

  void startXYZ() {
    printf("startXYZ\n");
    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
        boost::bind (&CloudGrabberImpl::xyz_cb, this, _1);
    c = m_interface.registerCallback (f);
    m_interface.start();
  }

  void stop() {
    m_interface.stop();

  }
  ~CloudGrabberImpl() {stop();}

};

CloudGrabber::CloudGrabber() {
  m_impl.reset(new CloudGrabberImpl());
}


pcl::PointCloud<pcl::PointXYZ>::Ptr CloudGrabber::getXYZ() {
  return m_impl->getXYZ();
}

void CloudGrabber::startXYZ() {
  m_impl->startXYZ();
}

void CloudGrabber::stop() {
  m_impl->stop();
}


}

