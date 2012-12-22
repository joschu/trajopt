#pragma once
#include "typedefs.hpp"
#include <openrave/openrave.h>

namespace trajopt {

class RobotAndDOF {
public:
  RobotAndDOF(OR::RobotBasePtr _robot, const IntVec& _joint_inds, int _affinedofs=0, const OR::Vector _rotationaxis=OR::Vector()) :
    robot(_robot), joint_inds(_joint_inds), affinedofs(_affinedofs), rotationaxis(_rotationaxis) {}

  void SetDOFValues(const DblVec& dofs);
  void GetDOFLimits(DblVec& lower, DblVec& upper) const;
  int GetDOF() const;
  IntVec GetJointIndices() const {return joint_inds;}
  Matrix CalculatePositionJacobian() const;
  Matrix CalculateRotationJacobian() const;
  OR::RobotBasePtr GetRobot() const {return robot;}

private:
  OR::RobotBasePtr robot;
  IntVec joint_inds;
  int affinedofs;
  OR::Vector rotationaxis;
};

}
