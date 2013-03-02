#pragma once
#include "typedefs.hpp"
#include <openrave/openrave.h>

namespace trajopt {


/**
Stores an OpenRAVE robot and the active degrees of freedom  
*/
class TRAJOPT_API RobotAndDOF {
public:
  RobotAndDOF(OR::RobotBasePtr _robot, const IntVec& _joint_inds, int _affinedofs=0, const OR::Vector _rotationaxis=OR::Vector()) :
    robot(_robot), joint_inds(_joint_inds), affinedofs(_affinedofs), rotationaxis(_rotationaxis) {}

  void SetDOFValues(const DblVec& dofs);
  void GetDOFLimits(DblVec& lower, DblVec& upper) const;
  DblVec GetDOFValues();
  int GetDOF() const;
  IntVec GetJointIndices() const {return joint_inds;}
  DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const;
  DblMatrix RotationJacobian(int link_ind) const;
  OR::RobotBasePtr GetRobot() const {return robot;}
  OR::RobotBase::RobotStateSaver Save() {return OR::RobotBase::RobotStateSaver(robot, /*just save trans*/ 1);}
  bool DoesAffect(const KinBody::Link& link);
  std::vector<KinBody::LinkPtr> GetAffectedLinks();
  void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds);
  DblVec RandomDOFValues();
  void SetRobotActiveDOFs() {robot->SetActiveDOFs(joint_inds, affinedofs);}
private:
  OR::RobotBasePtr robot;
  IntVec joint_inds;
  int affinedofs;
  OR::Vector rotationaxis;
};
typedef boost::shared_ptr<RobotAndDOF> RobotAndDOFPtr;

}
