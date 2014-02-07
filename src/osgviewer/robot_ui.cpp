#include "robot_ui.hpp"
using namespace osg;
using namespace std;
namespace OR = OpenRAVE;
#include <boost/foreach.hpp>
#include "utils/stl_to_string.hpp"
#include <osg/io_utils>
#include "utils/logging.hpp"
#include "utils/eigen_conversions.hpp"
#include <Eigen/Dense>
using namespace util;
using namespace OpenRAVE;
namespace {
template <class T>
osg::Vec3 toOsgVec3(const OR::RaveVector<T>& v) {
  return Vec3(v.x, v.y, v.z);
}
template <class T>
osg::Vec4 toOsgVec4(const OR::RaveVector<T>& v) {
  return Vec4(v.x, v.y, v.z, v.w);
}

}

OpenRAVE::Vector toRave(const osg::Vec3& v) {
  return OpenRAVE::Vector(v.x(), v.y(), v.z());
}
OpenRAVE::Vector toRave(const osg::Quat& q) {
  return OpenRAVE::Vector(q.w(), q.x(), q.y(), q.z());
}
vector<double> GetDOFValues(const OR::RobotBase::Manipulator& manip) {
  vector<double> vals;
  manip.GetRobot()->GetDOFValues(vals, manip.GetArmIndices());
  return vals;
}


ManipulatorControl::ManipulatorControl(OpenRAVE::RobotBase::ManipulatorPtr manip, OSGViewerPtr viewer) :
      m_manip(manip), m_viewer(viewer) {
  viewer->AddMouseCallback(boost::bind(&ManipulatorControl::ProcessMouseInput, this, _1));
}

bool ManipulatorControl::ProcessMouseInput(const osgGA::GUIEventAdapter &ea) {

  if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {
    lastX = ea.getX();
    lastY = ea.getY();
  }
  else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG) {
    const bool ctrl = ( ea.getModKeyMask() &   (osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL  | osgGA::GUIEventAdapter::MODKEY_LEFT_SUPER) ) != 0 ;
    if (!ctrl) return false;
    // drag the active manipulator in the plane of view
    // get our current view

    float dx = ea.getX() - lastX;
    lastX = ea.getX();
    float dy = ea.getY() - lastY;
    lastY = ea.getY();




    Vec3d from, to, up;
    m_viewer->m_handler->getTransformation(from, to, up);
    up.normalize();
    Vec3d depthdir = (to - from); depthdir.normalize();
    osg::Vec3 ydir = up;
    osg::Vec3 xdir = depthdir ^ ydir;

    double fovy, aspectRatio, Zn, Zf;
    m_viewer->m_cam->getProjectionMatrixAsPerspective( fovy, aspectRatio, Zn, Zf );
    fovy *= M_PI/180; 
    double fovx = fovy * aspectRatio;

    OpenRAVE::Transform T = m_manip->GetEndEffectorTransform();
    float depth = (toOsgVec3(T.trans)-from) * depthdir;
    // cout << depth << " " << dx << " " << ea.getWindowWidth() << " | " << xdir << endl;
    osg::Vec3 dragvec = xdir*(depth*dx/ea.getWindowWidth()*fovx) + ydir*(depth*dy/ea.getWindowHeight()*fovy);


#if 1
  vector<double> _jac;
  RobotBasePtr robot = m_manip->GetRobot();
  vector<int> dofinds = m_manip->GetArmIndices();
  if (ea.getModKeyMask() & (osgGA::GUIEventAdapter::MODKEY_CTRL)) {    
    robot->ComputeJacobianTranslation(m_manip->GetEndEffector()->GetIndex(), m_manip->GetEndEffector()->GetTransform().trans, _jac, dofinds);
    Eigen::MatrixXd jac = Eigen::Map<Eigen::MatrixXd>(_jac.data(), 3, dofinds.size());
    Eigen::VectorXd djoints = jac.colPivHouseholderQr().solve(Eigen::Vector3d(dragvec.x(), dragvec.y(), dragvec.z()));
    vector<double> newjoints;
    robot->GetDOFValues(newjoints, dofinds);
    for (int i=0; i < newjoints.size(); ++i) newjoints[i] += djoints[i];
    robot->SetDOFValues(newjoints, true, dofinds);
  }
  else {
    OpenRAVE::Vector axis = toRave(dragvec ^ depthdir);
    float angle = dragvec.length()*10;

    robot->ComputeJacobianAxisAngle(m_manip->GetEndEffector()->GetIndex(), _jac, dofinds);
    Eigen::MatrixXd jac = Eigen::Map<Eigen::MatrixXd>(_jac.data(), 3, dofinds.size());
    Eigen::Vector3d rotvec(axis.x, axis.y, axis.z);
    rotvec *= angle;
    Eigen::VectorXd djoints = jac.colPivHouseholderQr().solve(rotvec);
    vector<double> newjoints;
    robot->GetDOFValues(newjoints, dofinds);
    for (int i=0; i < newjoints.size(); ++i) newjoints[i] += djoints[i];
    robot->SetDOFValues(newjoints, true, dofinds);
  }

#else
    if (ea.getModKeyMask() & (osgGA::GUIEventAdapter::MODKEY_CTRL)) {
      T.trans += toRave(dragvec);      
    }
    else {
      osg::Vec3d axis = dragvec ^ depthdir;
      float angle = dragvec.length()*3;
      osg::Quat rot(angle, axis);
      T.rot = quatMultiply(toRave(rot),T.rot);
      // if (rot.length() > 0.99f && rot.length() < 1.01f)
      //     newTrans.setRotation(rot * origTrans.getRotation());      
    }  
    vector<double> iksoln;
    m_manip->FindIKSolution(OR::IkParameterization(T), iksoln, 18);
    if (iksoln.empty()) {
      LOG_INFO("no ik solution found");
    }
    else {
      LOG_INFO("ik succeeded!");
      m_manip->GetRobot()->SetDOFValues(iksoln, false, m_manip->GetArmIndices());
    }
#endif
    m_viewer->UpdateSceneData();
    return true;
  }
  return false;

}

DriveControl::DriveControl(OpenRAVE::RobotBasePtr robot, OSGViewerPtr viewer) :
        m_robot(robot), m_viewer(viewer) {
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Left, boost::bind(&DriveControl::MoveRobot, this, 0,.05, 0));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Right, boost::bind(&DriveControl::MoveRobot,this, 0,-.05, 0));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Up, boost::bind(&DriveControl::MoveRobot, this,.05,0, 0));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Down, boost::bind(&DriveControl::MoveRobot, this,-.05,0, 0));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Leftbracket, boost::bind(&DriveControl::MoveRobot, this,0,0, .05));
  viewer->AddKeyCallback(osgGA::GUIEventAdapter::KEY_Rightbracket, boost::bind(&DriveControl::MoveRobot, this,0,0, -.05));
}

void DriveControl::MoveRobot(float dx, float dy, float dtheta) {
  OR::Transform T = m_robot->GetTransform();
  T.trans += OpenRAVE::Vector(dx, dy, 0);
  T.rot = OpenRAVE::geometry::quatMultiply(T.rot, OpenRAVE::Vector(1,0,0,dtheta/2));
  T.rot.normalize4();
  cout << "rotation " << T.rot << endl;
  m_robot->SetTransform(T);
  m_viewer->UpdateSceneData();
}

void StatePrinter::PrintAll() {
  cout << "joints: " << endl;
  BOOST_FOREACH(const OR::KinBody::JointPtr& joint, m_robot->GetJoints()) {
    cout << joint->GetName() << ": " << Str(joint->GetValue(0)) << endl;
  }
  vector<double> dofvals; m_robot->GetDOFValues(dofvals);
  cout << "all dof vals: " << Str(dofvals) << endl;
  cout << "transform: " << m_robot->GetTransform() << endl;
  cout << "links: " << endl;
  BOOST_FOREACH(const OR::KinBody::LinkPtr& link, m_robot->GetLinks()) {
    cout << link->GetName() << ": " << link->GetTransform() << endl;
  }
}



InteractiveMarker::InteractiveMarker(const OpenRAVE::Transform& initTrans, OSGViewerPtr viewer) :
  m_viewer(viewer), m_T(initTrans) {
  viewer->AddMouseCallback(boost::bind(&InteractiveMarker::ProcessMouseInput, this, _1));
}

bool InteractiveMarker::ProcessMouseInput(const osgGA::GUIEventAdapter &ea) {

  if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH) {
    lastX = ea.getX();
    lastY = ea.getY();
  }
  else if (ea.getEventType() == osgGA::GUIEventAdapter::DRAG) {
    const bool ctrl = ( ea.getModKeyMask() &   (osgGA::GUIEventAdapter::MODKEY_LEFT_CTRL  | osgGA::GUIEventAdapter::MODKEY_LEFT_SUPER) ) != 0 ;
    if (!ctrl) return false;
    // drag the active manipulator in the plane of view
    // get our current view

    float dx = ea.getX() - lastX;
    lastX = ea.getX();
    float dy = ea.getY() - lastY;
    lastY = ea.getY();




    Vec3d from, to, up;
    m_viewer->m_handler->getTransformation(from, to, up);
    up.normalize();
    Vec3d depthdir = (to - from); depthdir.normalize();
    osg::Vec3 ydir = up;
    osg::Vec3 xdir = depthdir ^ ydir;

    double fovy, aspectRatio, Zn, Zf;
    m_viewer->m_cam->getProjectionMatrixAsPerspective( fovy, aspectRatio, Zn, Zf );
    fovy *= M_PI/180; 
    double fovx = fovy * aspectRatio;

    float depth = (toOsgVec3(m_T.trans)-from) * depthdir;
    // cout << depth << " " << dx << " " << ea.getWindowWidth() << " | " << xdir << endl;
    osg::Vec3 dragvec = xdir*(depth*dx/ea.getWindowWidth()*fovx) + ydir*(depth*dy/ea.getWindowHeight()*fovy);


    if (ea.getModKeyMask() & (osgGA::GUIEventAdapter::MODKEY_CTRL)) {
      m_T.trans += toRave(dragvec);      
    }
    else {
      osg::Vec3d axis = dragvec ^ depthdir;
      float angle = dragvec.length()*3;
      osg::Quat rot(angle, axis);
      m_T.rot = quatMultiply(toRave(rot),m_T.rot);
      // if (rot.length() > 0.99f && rot.length() < 1.01f)
      //     newTrans.setRotation(rot * origTrans.getRotation());      
    }  
    m_viewer->UpdateSceneData();
    return true;
  }
  return false;

}