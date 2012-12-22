#include <gtest/gtest.h>
#include <openrave-core.h>
#include "trajopt/collision_checker.hpp"
#include "utils/general_utils.hpp"
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
  RaveInitialize(false);
  RaveSetDebugLevel(Level_Debug);
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


  checker->SetContactDistance(0);
  box1->SetTransform(Transform(Vector(1,0,0,0), Vector(.9,0,0)));
  box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
  {
    vector<Collision> collisions;
    checker->AllVsAll(NULL, collisions);
    PrintCollisions(collisions);
  }


  checker->SetContactDistance(0);
  box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
  box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
  {
    vector<Collision> collisions;
    checker->AllVsAll(NULL, collisions);
    PrintCollisions(collisions);
  }

  checker->SetContactDistance(.05);
  box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
  box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
  {
    vector<Collision> collisions;
    checker->AllVsAll(NULL, collisions);
    PrintCollisions(collisions);
  }



  checker->SetContactDistance(.2);
  box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
  box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
  {
    vector<Collision> collisions;
    checker->AllVsAll(NULL, collisions);
    PrintCollisions(collisions);
  }



}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
