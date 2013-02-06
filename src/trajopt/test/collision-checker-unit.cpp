#include <gtest/gtest.h>
#include <openrave-core.h>
#include "trajopt/collision_checker.hpp"
#include "utils/stl_to_string.hpp"
#include "utils/eigen_conversions.hpp"
using namespace OpenRAVE;
using namespace std;
using namespace trajopt;
using namespace util;

string data_dir() {
  string out = DATA_DIR;
  return out;
}



void PrintCollisions(const vector<Collision>& collisions) {
  RAVELOG_INFO("%i collisions found\n", collisions.size());
  for (int i=0; i < collisions.size(); ++i) {
    const Collision& c = collisions[i];
    RAVELOG_INFO("%i: bodies: %s-%s. normal: %s. ptA: %s. distance: %.3e\n", i, c.linkA->GetName().c_str(), c.linkB->GetName().c_str(),
      Str(c.normalB2A).c_str(), Str(c.ptA).c_str(), c.distance);
  }
}



TEST(collision_checker, box_distance) {
  EnvironmentBasePtr env = RaveCreateEnvironment();
  ASSERT_TRUE(env->Load(data_dir() + "/box.xml"));
  KinBodyPtr box0 = env->GetKinBody("box");
  box0->SetName("box0");
  ASSERT_TRUE(env->Load(DATA_DIR + string("/box.xml")));
  KinBodyPtr box1 = env->GetKinBody("box");
  box1->SetName("box1");
  vector<KinBodyPtr> bodies; env->GetBodies(bodies);
  RAVELOG_DEBUG("%i kinbodies in rave env\n", bodies.size());
  CollisionCheckerPtr checker = CreateCollisionChecker(env);

#define EXPECT_NUM_COLLISIONS(n)\
  vector<Collision> collisions;\
  checker->AllVsAll(collisions);\
  EXPECT_EQ(collisions.size(), n);\
  collisions.clear();\
  checker->BodyVsAll(*box0, collisions);\
  EXPECT_EQ(collisions.size(), n);

  {
    checker->SetContactDistance(0);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(.9,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(1);
  }

  {
    checker->SetContactDistance(0);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(0);
  }

  {
    checker->SetContactDistance(.04);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(0);
  }

  {
    checker->SetContactDistance(.1);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(1);
  }


  {
    checker->SetContactDistance(.2);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(1);
  }

  {
    env->Remove(box1);
    EXPECT_NUM_COLLISIONS(0);
  }


}

TEST(continuous_collisions, boxes) {
  EnvironmentBasePtr env = RaveCreateEnvironment();
  ASSERT_TRUE(env->Load(data_dir() + "/box.xml"));
  ASSERT_TRUE(env->Load(data_dir() + "/boxbot.xml"));
  KinBodyPtr box = env->GetKinBody("box");
  RobotBasePtr boxbot = env->GetRobot("boxbot");

  CollisionCheckerPtr checker = CreateCollisionChecker(env);
  {
    RobotAndDOFPtr rad(new RobotAndDOF(boxbot, IntVec(), DOF_X | DOF_Y, Vector()));
    TrajArray traj(2,2);
    traj << -1.9,0,  0,1.9;
    vector<Collision> collisions;
    checker->ContinuousCheckTrajectory(traj, *rad, collisions);
    ASSERT_EQ(collisions.size(), 1);
    Collision col = collisions[0];
    EXPECT_NEAR(col.normalB2A.x, -1, 1e-4);
    EXPECT_NEAR(col.normalB2A.y, 0, 1e-4);
    EXPECT_NEAR(col.normalB2A.z, 0, 1e-4);
  }
  {
    RobotAndDOFPtr rad(new RobotAndDOF(boxbot, IntVec(), DOF_X | DOF_Y, Vector()));
    TrajArray traj(2,2);
    traj << -1.9,0,  0,1.9;
    vector<Collision> collisions;
    checker->CastVsAll(*rad, rad->GetRobot()->GetLinks(), toDblVec(traj.row(0)), toDblVec(traj.row(1)), collisions);
    ASSERT_EQ(collisions.size(), 1);
    Collision col = collisions[0];
    cout << col << endl;
//    EXPECT_NEAR(col.normalB2A.x, , 1e-4);
//    EXPECT_NEAR(col.normalB2A.y, 0, 1e-4);
//    EXPECT_NEAR(col.normalB2A.z, 0, 1e-4);
  }


}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  RaveInitialize(false);
  RaveSetDebugLevel(Level_Debug);

  return RUN_ALL_TESTS();
}
