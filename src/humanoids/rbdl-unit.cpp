#include <gtest/gtest.h>
#include "humanoids/rave_to_rbdl.hpp"
#include <openrave-core.h>
#include "trajopt/robot_and_dof.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include <boost/foreach.hpp>
#include "utils/eigen_conversions.hpp"
#include <iostream>
using namespace std;
using namespace OpenRAVE;
using namespace trajopt;
using namespace util;

TEST(rbdl, main) {
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  env->Load("robots/puma.robot.xml");
  RobotBasePtr robot = GetRobot(*env);
  vector<int> inds;
  for (int i=0; i < robot->GetDOF(); ++i) inds.push_back(i);
  RobotAndDOFPtr rad(new RobotAndDOF(robot, inds));
  DblVec dofvals = rad->RandomDOFValues();
//  rad->SetDOFValues(dofvals);


  robot->SetDOFValues(dofvals,false);

  std::map<KinBody::LinkPtr, unsigned> link2id;
  boost::shared_ptr<rbd::Model> model = MakeRBDLModel(robot, link2id);
  rbd::Math::VectorNd Q = toVectorXd(dofvals);
  rbd::UpdateKinematicsCustom(*model, &Q, NULL, NULL);

  BOOST_FOREACH(KinBody::JointPtr joint, robot->GetJoints()) {
    KinBody::LinkPtr childLink = joint->GetHierarchyChildLink();
    int i = link2id[childLink];
    cout << childLink->GetName() << endl;
    cout << (toRave(model->X_base[i]) * joint->GetInternalHierarchyRightTransform()).trans
        << " =?= " << toVector3d(childLink->GetTransform().trans).transpose() << endl;
    cout << toMatrix3d(toRave(model->X_base[i]) * joint->GetInternalHierarchyRightTransform()) << " =?= \n"
        << toMatrix3d(childLink->GetTransform()) << endl;

  }



}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  RaveInitialize(false);

  return RUN_ALL_TESTS();
}
