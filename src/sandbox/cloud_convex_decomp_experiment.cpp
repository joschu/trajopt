#include "cloudproc/convexdecomp.hpp"
#include "cloudproc/cloudproc.hpp"
#include "cloudproc/sphere_sampling.hpp"
#include <boost/foreach.hpp>
#include "LinearMath/btConvexHull.h"
#include <openrave/openrave.h>
#include <openrave-core.h>
#include "osgviewer/osgviewer.hpp"
using namespace pcl;
using namespace OpenRAVE;
using namespace std;
using namespace cloudproc;

#ifdef __CDT_PARSER__
#define BOOST_FOREACH(...) for(;;)
#endif

btVector3 toBt(const Vector& v) {
  return btVector3(v.x, v.y, v.z);
}
Vector toOR(const btVector3& v) {
  return Vector(v.x(), v.y(), v.z());
}
btVector3 toBt(const PointXYZ& v) {
  return btVector3(v.x, v.y, v.z);
}
ostream &operator<<(ostream &stream, const btVector3& v) {
  stream << v.x() << " " << v.y() << " " << v.z();
  return stream;
}

int main() {

  RaveInitialize(true, OpenRAVE::Level_Debug);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  bool success = env->Load("robots/pr2-beta-static.zae");
  OSGViewerPtr viewer(new OSGViewer(env));


  vector<RobotBasePtr> robots;
  env->GetRobots(robots);
  RobotBasePtr robot = robots[0];
  Vector cam = robot->GetLink("laser_tilt_link")->GetTransform().trans;

  PointCloud<PointXYZ>::Ptr cloud = readPCDXYZ(
      "/home/joschu/Downloads/laser_cloud.pcd");
  cloud = downsampleCloud(cloud, .02);


  vector<IntVec> hull_inds;
  ConvexDecomp(cloud, getSpherePoints(1), .02, NULL, &hull_inds);

  float extrusion_dist = .5;
  vector<GraphHandlePtr> handles;

  BOOST_FOREACH(const IntVec& inds, hull_inds) {
    vector<btVector3> origpts;
    BOOST_FOREACH(int ind, inds) {
      origpts.push_back(toBt(cloud->points[ind]));
      origpts.push_back(origpts.back() + extrusion_dist * (origpts.back() - toBt(cam)).normalized());
    }
    cout << "num pts: " << origpts.size() << endl;
    if (origpts.size() < 5) continue;
    HullDesc hd;
    hd.mFlags = QF_TRIANGLES;
    hd.mVcount = origpts.size();
    hd.mVertices = &origpts[0];
    hd.mVertexStride = sizeof(btVector3);

    HullLibrary hl;
    HullResult hr;


    if (hl.CreateConvexHull(hd, hr) == QE_FAIL) {
      continue;
    }

#define randf() ((float)rand())/RAND_MAX

    handles.push_back(viewer->drawtrimesh((float*) &hr.m_OutputVertices[0], 16,
        (int*) &hr.m_Indices[0], hr.mNumFaces, RaveVectorf(randf(), randf(), randf(), 1)));
    SetTransparency(handles.back(), 1);

  }


  Eigen::MatrixXf colors(cloud->size(), 4);
  colors.setOnes();
  handles.push_back(viewer->plot3((float*)cloud->points.data(), cloud->size(), 16, 8, colors.data()));

  viewer->Idle();

}
