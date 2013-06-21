#include <gtest/gtest.h>
#include "humanoids/rave_to_rbdl.hpp"
#include <openrave-core.h>
#include "trajopt/configuration_space.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include <boost/foreach.hpp>
#include "utils/eigen_conversions.hpp"
#include "utils/stl_to_string.hpp"
#include <iostream>
#include "osgviewer/osgviewer.hpp"
#include <Eigen/Geometry>
#include <rbdl_utils.h>

using namespace std;
using namespace OpenRAVE;
using namespace trajopt;
using namespace util;
using namespace Eigen;

Vector3d axisAngleToEulerZYX(const Vector3d& in) {
  double rx = in(0), ry = in(1), rz = in(2);
  Matrix3d mat = toMatrix3d(OpenRAVE::geometry::matrixFromAxisAngle(OpenRAVE::Vector(rx, ry, rz)));
  return mat.eulerAngles(2,1,0);
}
Vector3d eulerZYXtoAxisAngle(const Vector3d& in) {
  double rz = in(0), ry = in(1), rx = in(2);
  Matrix3d mat; mat = AngleAxisd(rz, Vector3d::UnitZ())
               * AngleAxisd(ry, Vector3d::UnitY())
               * AngleAxisd(rx, Vector3d::UnitX());
  OpenRAVE::TransformMatrix tm;
  tm.rotfrommat(mat(0,0), mat(0,1), mat(0,2), mat(1,0), mat(1,1), mat(1,2), mat(2,0), mat(2,1), mat(2,2));
  return toVector3d(OpenRAVE::geometry::axisAngleFromMatrix(tm));

}

VectorXd toRBDLOrder(const DblVec& _x) {
  VectorXd x = toVectorXd(_x);
  int n = x.size();
  VectorXd y(n);
  y.topRows(3) = x.middleRows(n-6,3);
  y.middleRows(3,3) = axisAngleToEulerZYX(x.bottomRows(3));
  y.bottomRows(n-6) = x.topRows(n-6);
  return y;
}
DblVec toRaveOrder(const VectorXd& x) {
  int n = x.size();
  VectorXd y(x.size());
  y.topRows(n-6) = x.bottomRows(n-6);
  y.middleRows(n-6,3) = x.topRows(3);
  y.bottomRows(3) = eulerZYXtoAxisAngle(x.middleRows(3,3));
  return toDblVec(y);
}


#define EXPECT_MATRIX_NEAR(_mat0, _mat1, abstol) do {\
    Eigen::MatrixXd mat0=_mat0, mat1=_mat1;\
    ASSERT_TRUE(mat0.rows() == mat1.rows());\
    ASSERT_TRUE(mat0.cols() == mat1.cols());\
    bool fail = false;\
    for (int i=0; i < mat0.rows(); ++i) {\
      for (int j=0; j < mat0.cols(); ++j) {\
        if (fabs(mat0(i,j) - mat1(i,j)) > abstol) {\
          fail = true;\
        }\
      }\
    }\
    if (fail) {\
      char msg[1000];\
      sprintf(msg, "%s !=\n %s    (tol %.2e) at %s:%i\n", CSTR(mat0), CSTR(mat1), abstol, __FILE__, __LINE__);\
      GTEST_NONFATAL_FAILURE_(msg);\
    }\
} while(0)

TEST(math, rotation_conversion) {
  {
    Vector3d x = Vector3d::Random();
    EXPECT_MATRIX_NEAR(x, axisAngleToEulerZYX(eulerZYXtoAxisAngle(x)),1e-6);
    EXPECT_MATRIX_NEAR(x, eulerZYXtoAxisAngle(axisAngleToEulerZYX(x)),1e-6);
  }
  {
    VectorXd x = VectorXd::Random(40);
    EXPECT_MATRIX_NEAR(x, toVectorXd(toRaveOrder(toRBDLOrder(toDblVec(x)))),1e-6);
    EXPECT_MATRIX_NEAR(x, toRBDLOrder(toRaveOrder(x)),1e-6);
  }
}

TEST(fixed_base, kinematics) {
  /**
   *
   * Check to see that OpenRAVE kinematics gives the same result as RBDL kinematics on fixed-base robot
   */
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  env->Load("robots/pr2-beta-static.zae");
  RobotBasePtr robot = GetRobot(*env);
  vector<int> inds;
  for (int i=0; i < robot->GetDOF(); ++i) inds.push_back(i);
  RobotAndDOFPtr rad(new RobotAndDOF(robot, inds));
  DblVec dofvals = rad->RandomDOFValues();


  robot->SetDOFValues(dofvals,true);

  std::map<KinBody::LinkPtr, unsigned> link2id;
  boost::shared_ptr<rbd::Model> model = MakeRBDLModel(robot, false, link2id);
  rbd::Math::VectorNd Q = toVectorXd(dofvals);
  rbd::UpdateKinematicsCustom(*model, &Q, NULL, NULL);

  cout << "model hier\n" << rbd::Utils::GetModelHierarchy (*model) << endl;
  cout << "model dof overview\n" << rbd::Utils::GetModelDOFOverview (*model) << endl;
  cout << "named bodies\n" << rbd::Utils::GetNamedBodyOriginsOverview(*model) << endl;

  BOOST_FOREACH(KinBody::JointPtr joint, robot->GetJoints()) {
    KinBody::LinkPtr childLink = joint->GetHierarchyChildLink();
    int i = link2id[childLink];
    cout << childLink->GetName() << " " << childLink->GetTransform() <<  endl;
    DblVec vals;
    joint->GetValues(vals);
    cout << "vasls: " << vals[0] << endl;
    EXPECT_MATRIX_NEAR(toVector3d((toRave(model->X_base[i]) * joint->GetInternalHierarchyRightTransform()).trans),
        toVector3d(childLink->GetTransform().trans), 1e-4);
    EXPECT_MATRIX_NEAR(toMatrix3d(toRave(model->X_base[i]) * joint->GetInternalHierarchyRightTransform()),
        toMatrix3d(childLink->GetTransform()), 1e-4);

  }
  RaveDestroy();
}


TEST(floating_base, kinematics) {

  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  env->Load(string(BIGDATA_DIR) + "/atlas.xml");
  RobotBasePtr robot = GetRobot(*env);
  DblVec lower(robot->GetDOF(), -1000);
  DblVec upper(robot->GetDOF(), 1000);
  robot->SetDOFLimits(lower, upper);


  vector<int> inds;
  for (int i=0; i < robot->GetDOF(); ++i) inds.push_back(i);
  RobotAndDOFPtr rad(new RobotAndDOF(robot, inds, OpenRAVE::DOF_XYZ | OpenRAVE::DOF_Rotation3D));
  DblVec dofvals = rad->RandomDOFValues();
  rad->SetDOFValues(dofvals);

  std::map<KinBody::LinkPtr, unsigned> link2id;
  boost::shared_ptr<rbd::Model> model = MakeRBDLModel(robot, true, link2id);
  rbd::Math::VectorNd Q = toRBDLOrder(dofvals);
  rbd::UpdateKinematicsCustom(*model, &Q, NULL, NULL);

  cout << Q.transpose() << endl;
  cout << Str(rad->GetDOFValues()) << endl;

  BOOST_FOREACH(KinBody::JointPtr joint, robot->GetJoints()) {
    KinBody::LinkPtr childLink = joint->GetHierarchyChildLink();
    int i = link2id[childLink];
    cout << childLink->GetName() << endl;
    EXPECT_MATRIX_NEAR(toVector3d((toRave(model->X_base[i]) * joint->GetInternalHierarchyRightTransform()).trans),
        toVector3d(childLink->GetTransform().trans), 1e-4);
    EXPECT_MATRIX_NEAR(toMatrix3d(toRave(model->X_base[i]) * joint->GetInternalHierarchyRightTransform()),
        toMatrix3d(childLink->GetTransform()), 1e-4);
  }
  RaveDestroy();
}




TEST(fixed_base, dynamics) {
  /**
   *
   * Simulate fixed-base robot
   */
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  env->Load("robots/puma.robot.xml");
  RobotBasePtr robot = GetRobot(*env);

  std::map<KinBody::LinkPtr, unsigned> link2id;
  boost::shared_ptr<rbd::Model> model = MakeRBDLModel(robot, false, link2id);

  VectorXd Q = VectorXd::Zero(model->dof_count);
  VectorXd QD = VectorXd::Zero(model->dof_count);
  VectorXd QDD = VectorXd::Zero(model->dof_count);
  VectorXd Tau = VectorXd::Zero(model->dof_count);

  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);


  float dt = .01;
  for (int i=0; i < 100; ++i) {
    rbd::ForwardDynamics(*model, Q, QD, Tau, QDD, NULL);
    Q += QD * dt;
    QD += QDD * dt;
    robot->SetDOFValues(toDblVec(Q), false);
    viewer->Idle();
  }
  RaveDestroy();


}



TEST(floating_base, dynamics) {
  /**
   *
   * Simulate fixed-base robot
   */
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  env->Load(string(BIGDATA_DIR)+"/atlas.xml");
  RobotBasePtr robot = GetRobot(*env);
  DblVec lower(robot->GetDOF(), -10000);
  DblVec upper(robot->GetDOF(), 10000);
  robot->SetDOFLimits(lower, upper);

  vector<int> inds;
  for (int i=0; i < robot->GetDOF(); ++i) inds.push_back(i);
  RobotAndDOFPtr rad(new RobotAndDOF(robot, inds, OpenRAVE::DOF_XYZ | OpenRAVE::DOF_Rotation3D));

  std::map<KinBody::LinkPtr, unsigned> link2id;
  boost::shared_ptr<rbd::Model> model = MakeRBDLModel(robot, true, link2id);

  VectorXd Q = VectorXd::Zero(model->dof_count);
  VectorXd QD = VectorXd::Zero(model->dof_count);
  VectorXd QDD = VectorXd::Zero(model->dof_count);
  VectorXd Tau = VectorXd::Zero(model->dof_count);

  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);

  vector<rbdmath::SpatialVector> f_ext(model->f.size(), rbdmath::SpatialVectorZero);

  KinBody::LinkPtr l_foot = robot->GetLink("l_foot");

  float dt = .005;
  for (int i=0; i < 1000; ++i) {

    OpenRAVE::Transform body_frame = robot->GetTransform();
    Vector3d l_foot_pos = toVector3d( (body_frame.inverse()*l_foot->GetTransform()).trans);
    Vector3d l_foot_force = toMatrix3d(body_frame).inverse() * Vector3d(0,0,10);
    cout << l_foot_force.transpose() << endl;
    cout << l_foot_pos.transpose() << endl;
//    f_ext[link2id[l_foot]] = concat(l_foot_force, l_foot_force.cross(l_foot_pos));
    Vector3d l_foot_force2(0,0,10);
    f_ext[link2id[l_foot]] = concat(l_foot_force2,toVector3d(l_foot->GetTransform().trans).cross( l_foot_force2)) ;


    Tau = -QD*.1;
    rbd::ForwardDynamics(*model, Q, QD, Tau, QDD, &f_ext);
    Q += QD * dt;
    QD += QDD * dt;
    rad->SetDOFValues(toRaveOrder(Q));
    viewer->Idle();
  }
  RaveDestroy();


}

TEST(fixed_base, invdynamics) {

  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  env->Load("robots/puma.robot.xml");
  RobotBasePtr robot = GetRobot(*env);

  std::map<KinBody::LinkPtr, unsigned> link2id;
  boost::shared_ptr<rbd::Model> model = MakeRBDLModel(robot, false, link2id);

  VectorXd Q = VectorXd::Zero(robot->GetDOF());
  VectorXd QDot = VectorXd::Zero(robot->GetDOF());
  VectorXd QDDot = VectorXd::Zero(robot->GetDOF());
  VectorXd Tau = VectorXd::Zero(robot->GetDOF());
  rbd::InverseDynamics(*model, Q, QDot, QDDot, Tau, NULL);
  DblVec tau_rave;
  robot->SetDOFVelocities(toDblVec(QDot));
  robot->ComputeInverseDynamics(tau_rave, toDblVec(QDDot));
  EXPECT_MATRIX_NEAR(toVectorXd(tau_rave), Tau, 1e-4);
}


int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  RaveInitialize(false);

  return RUN_ALL_TESTS();
}
