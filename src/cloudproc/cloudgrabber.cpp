#include "cloudgrabber.hpp"
#include <pcl/io/openni_grabber.h>
#include <boost/function.hpp>
#include <cstdio>
using namespace pcl;

namespace cloudproc {

class CloudGrabberImpl{
public:
  enum StreamingState {
    IDLE,
    XYZ,
    XYZRGB
  };
  enum RequestState {
    NONE,
    PENDING
  };
  PointCloud<PointXYZ>::Ptr m_xyz;
  PointCloud<PointXYZRGB>::Ptr m_xyzrgb;
  pcl::OpenNIGrabber m_interface;
  boost::signals2::connection c;
  RequestState m_rs;
  StreamingState m_ss;

  CloudGrabberImpl() : m_rs(NONE), m_ss(IDLE) {}

  void xyzrgb_cb (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud) {
    if (m_rs == PENDING) {
      if (!m_xyzrgb) m_xyzrgb.reset(new PointCloud<PointXYZRGB>());
      *m_xyzrgb = *cloud;
      m_rs = NONE;
    }
  }
  void xyz_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
    if (m_rs == PENDING) {
      if (!m_xyz) m_xyz.reset(new PointCloud<PointXYZ>());
      *m_xyz = *cloud;
      m_rs = NONE;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZ();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getXYZRGB();

  void startXYZ();
  void startXYZRGB();

  void stop() {
    m_interface.stop();
    m_ss = IDLE;

  }
  ~CloudGrabberImpl() {stop();}

};

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudGrabberImpl::getXYZ() {
  if (m_ss == IDLE) {
    printf("start streaming just for this point cloud\n");
    startXYZ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr out = getXYZ();
    stop();
    return out;
  }
  else if (m_ss == XYZ) {
    m_rs = PENDING;
    printf("waiting...\n");
    while (m_rs == PENDING) sleep(.01);
    printf("ok\n");
    return m_xyz;
  }
  else PRINT_AND_THROW("asked for xyz but currently streaming xyzrgb");
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudGrabberImpl::getXYZRGB() {
  if (m_ss == IDLE) {
    printf("start streaming just for this point cloud\n");
    startXYZRGB();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out = getXYZRGB();
    stop();
    return out;
  }
  else if (m_ss == XYZRGB) {
    m_rs = PENDING;
    while (m_rs == PENDING) sleep(.01);
    return m_xyzrgb;
  }
  else PRINT_AND_THROW("asked for xyzrgb but currently streaming xyz");
}
void CloudGrabberImpl::startXYZ() {
  printf("startxyz\n");
  m_ss = XYZ;
  boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
      boost::bind (&CloudGrabberImpl::xyz_cb, this, _1);
  c = m_interface.registerCallback (f);
  m_interface.start();
}
void CloudGrabberImpl::startXYZRGB() {
  m_ss = XYZRGB;
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
      boost::bind (&CloudGrabberImpl::xyzrgb_cb, this, _1);
  c = m_interface.registerCallback (f);
  m_interface.start();
}


CloudGrabber::CloudGrabber() {
  m_impl.reset(new CloudGrabberImpl());
}


pcl::PointCloud<pcl::PointXYZ>::Ptr CloudGrabber::getXYZ() {
  return m_impl->getXYZ();
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudGrabber::getXYZRGB() {
  return m_impl->getXYZRGB();
}

void CloudGrabber::startXYZ() {
  m_impl->startXYZ();
}
void CloudGrabber::startXYZRGB() {
  m_impl->startXYZRGB();
}

void CloudGrabber::stop() {
  m_impl->stop();
}


}



