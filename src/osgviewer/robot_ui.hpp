#pragma once
#include "osgviewer.hpp"

class ManipulatorControl {
public:
  ManipulatorControl(OpenRAVE::RobotBase::ManipulatorPtr manip, OSGViewerPtr viewer);
  bool ProcessMouseInput(const osgGA::GUIEventAdapter &ea);
private:
  OpenRAVE::RobotBase::ManipulatorPtr m_manip;
  OSGViewerPtr m_viewer;
  float lastX, lastY;
};

class DriveControl {
public:
  DriveControl(OpenRAVE::RobotBasePtr robot, OSGViewerPtr viewer);
  OpenRAVE::RobotBasePtr m_robot;
  OSGViewerPtr m_viewer;
  void MoveRobot(float dx, float dy, float dtheta);
};

class StatePrinter {
public:
  typedef OpenRAVE::RobotBasePtr RobotPtr;
  StatePrinter(RobotPtr robot) : m_robot(robot) {}
  void PrintAll();
private:
  RobotPtr m_robot;

};
