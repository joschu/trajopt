#include "cloudgrabber.hpp"
#include <pcl/io/openni_grabber.h>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>
#include <cstdio>
using namespace pcl;

namespace cloudproc {

class CloudGrabberImpl{
public:
  enum StreamingState {
    IDLE,
    XYZ,
    XYZRGB,
    RGBD
  };
  enum RequestState {
    NONE,
    PENDING
  };
  PointCloud<PointXYZ>::Ptr m_xyz;
  PointCloud<PointXYZRGB>::Ptr m_xyzrgb;
  RGBD::Ptr m_rgbd;
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
  
  void rgbd_cb(const boost::shared_ptr<openni_wrapper::Image>& rgb, const boost::shared_ptr<openni_wrapper::DepthImage>& depth) {
    if (m_rs == PENDING) {
      if (!m_rgbd) m_rgbd.reset(new cloudproc::RGBD());
      rgb->fillRGB(640, 480, m_rgbd->rgb.data(), 640*3);
      unsigned char* rgb_ptr = m_rgbd->rgb.data();
      unsigned char* rgb_end = m_rgbd->rgb.data() + m_rgbd->rgb.size();       
      unsigned short* depth_ptr = m_rgbd->depth.data();
      for (unsigned char* pix_ptr = rgb_ptr; pix_ptr < rgb_end; pix_ptr += 3) std::swap(pix_ptr[0], pix_ptr[2]);
      depth->fillDepthImageRaw(640, 480, depth_ptr);      
      m_rs = NONE;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr getXYZ();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getXYZRGB();
  RGBD::Ptr getRGBD();

  void startXYZ();
  void startXYZRGB();
  void startRGBD();

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
    while (m_rs == PENDING) boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    return m_xyz;
  }
  else PRINT_AND_THROW("asked for xyz but currently streaming another type");
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
    while (m_rs == PENDING) boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    return m_xyzrgb;
  }
  else PRINT_AND_THROW("asked for xyzrgb but currently streaming another type");
}
RGBD::Ptr CloudGrabberImpl::getRGBD() {
  if (m_ss == IDLE) {
    printf("start streaming just for this point cloud\n");
    startRGBD();
    RGBD::Ptr out = getRGBD();
    stop();
    return out;
  }
  else if (m_ss == RGBD) {
    m_rs = PENDING;
    while (m_rs == PENDING) boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    return m_rgbd;
  }
  else PRINT_AND_THROW("asked for rgbd but currently streaming another type");
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
void CloudGrabberImpl::startRGBD() {
  m_ss = RGBD;
  boost::function<void(const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float)> f = 
    boost::bind(&CloudGrabberImpl::rgbd_cb, this, _1, _2);
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
RGBD::Ptr CloudGrabber::getRGBD() {
  return m_impl->getRGBD();
}

void CloudGrabber::startXYZ() {
  m_impl->startXYZ();
}
void CloudGrabber::startXYZRGB() {
  m_impl->startXYZRGB();
}
void CloudGrabber::startRGBD() {
  m_impl->startRGBD();
}

void CloudGrabber::stop() {
  m_impl->stop();
}


}



