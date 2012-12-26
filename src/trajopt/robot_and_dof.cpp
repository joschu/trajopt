#include "robot_and_dof.hpp"
using namespace OpenRAVE;

namespace trajopt {

void RobotAndDOF::SetDOFValues(const DblVec& dofs) {
  if (affinedofs != 0) {
    OR::Transform T;
    OR::RaveGetTransformFromAffineDOFValues(T, dofs.begin()+joint_inds.size(), affinedofs, rotationaxis, true);
    robot->SetDOFValues(dofs, T, false);
  }
  else {
    robot->SetDOFValues(dofs, false);
  }
}

DblVec RobotAndDOF::GetDOFValues() {
  DblVec out;
  robot->GetDOFValues(out, joint_inds);
  if (affinedofs != 0) {
    out.resize(GetDOF());
    OR::Transform T = robot->GetTransform();
    OR::RaveGetAffineDOFValuesFromTransform(out.begin() + joint_inds.size(), T, affinedofs, rotationaxis);
  }
  return out;
}


void RobotAndDOF::GetDOFLimits(DblVec& lower, DblVec& upper) const {
  robot->GetDOFLimits(lower, upper, joint_inds);
  const int translation_dofs[3] = {DOF_X, DOF_Y, DOF_Z};
  for (int i=0; i < 3; ++i) {
    if (affinedofs & translation_dofs[i]) {
      lower.push_back(-INFINITY);
      upper.push_back(INFINITY);
    }
  }
  if (affinedofs & DOF_RotationMask) {
    if (affinedofs & DOF_RotationAxis) {
      lower.push_back(-INFINITY);
      upper.push_back(INFINITY);
    }
    else if (affinedofs & DOF_Rotation3D) {
      for (int i=0; i < 3; ++i) {
        lower.push_back(-INFINITY);
        upper.push_back(INFINITY);
      }
    }
    else if (affinedofs & DOF_Rotation3D) {
      for (int i=0; i < 3; ++i) {
        lower.push_back(-INFINITY);
        upper.push_back(INFINITY);
      }
    }
    else if (affinedofs & DOF_RotationQuat) {
      for (int i=0; i < 4; ++i) {
        lower.push_back(-1);
        upper.push_back(1);
      }
    }
    else throw OR::openrave_exception("invalid rotation dofs", ORE_InvalidArguments);
  }
}
int RobotAndDOF::GetDOF() const {
  return joint_inds.size() + RaveGetAffineDOF(affinedofs);
}
Matrix RobotAndDOF::CalculatePositionJacobian() const {
  throw OR::openrave_exception("not implemented", OR::ORE_NotImplemented);
}
Matrix RobotAndDOF::CalculateRotationJacobian() const {
  throw OR::openrave_exception("not implemented", OR::ORE_NotImplemented);
}


}
