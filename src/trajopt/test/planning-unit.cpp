#include <gtest/gtest.h>
#include <openrave-core.h>
#include <openrave/openrave.h>
#include "trajopt/collision_checker.hpp"
#include "utils/stl_to_string.hpp"
#include "trajopt/common.hpp"
#include "trajopt/problem_description.hpp"
#include "ipi/sco/optimizers.hpp"
#include "trajopt/rave_utils.hpp"
#include "osgviewer/osgviewer.h"
#include <ctime>
#include "utils/eigen_conversions.hpp"
#include "utils/clock.hpp"
#include <boost/foreach.hpp>
using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;

EnvironmentBase* gEnv;
OSGViewer* gViewer;

namespace {

OR::Transform randomReachablePose(RobotAndDOF& rad, KinBody::LinkPtr link) {
  // todo: save & restore
  DblVec dofvals = rad.RandomDOFValues();
  rad.SetDOFValues(dofvals);
  return link->GetTransform();
}

}

void PlotStateAndIdle(RobotAndDOF& rad, const DblVec& x) {
  rad.SetDOFValues(x);
  vector<GraphHandlePtr> handles;
  PlotAxes(*gEnv, rad.GetRobot()->GetLink("Puma6")->GetTransform(), .1, handles);
  gViewer->Idle();
}
void PlotCosts(vector<CostPtr>& costs, RobotAndDOF& rad, const DblVec& x) {
  rad.SetDOFValues(x);
  static vector<GraphHandlePtr> handles;
  handles.clear();
  BOOST_FOREACH(CostPtr& cost, costs) {
    if (Plottable* plotter = dynamic_cast<Plottable*>(cost.get())) {
      printf("plotting!\n");
      plotter->Plot(*gEnv, handles);
    }
  }
  gViewer->Idle();
}

void PlotTraj(RobotAndDOF& rad, const TrajArray& x, vector<GraphHandlePtr>& handles) {
  for (int i=0; i < x.rows(); ++i) {
    rad.SetDOFValues(toDblVec(x.row(i)));
    handles.push_back(gViewer->PlotKinBody(rad.GetRobot()));
    SetTransparency(handles.back(), .2);
  }
}

RobotBase::ManipulatorPtr GetOnlyManipulator(RobotBase& robot) {
  vector<RobotBase::ManipulatorPtr> manips = robot.GetManipulators();
  assert(manips.size()==1);
  return manips[0];
}

template<typename T>
void toJson(Json::Value& v, const T& x) {
  v.resize(x.size());
  for (int i=0; i < x.size(); ++i) {
    v[i] = x[i];
  }
}

TEST(numerical_ik, test1) {
  Json::Value root;
  Json::Reader reader;
  ifstream fh((string(DATA_DIR) + "/numerical_ik1.json").c_str());
  bool parse_success = reader.parse(fh, root);
  ASSERT_TRUE(parse_success);

  OR::RobotBasePtr robot = GetRobot(*gEnv);
  assert(!!robot);
  RobotBase::ManipulatorPtr manip = GetManipulatorByName(*robot, "arm");
  assert(!!manip);
  KinBody::LinkPtr link = robot->GetLink(root["costs"][0]["params"]["link"].asString());
  assert(!!link);
  RobotAndDOF rad(robot, manip->GetArmIndices());
  OR::Transform targ = randomReachablePose(rad, link);

  toJson(root["costs"][0]["params"]["xyz"], toVector3d(targ.trans));
  toJson(root["costs"][0]["params"]["wxyz"], toVector4d(targ.rot));

  vector<GraphHandlePtr> handles;
  PlotAxes(*gEnv, targ, .1, handles);

  ProblemConstructionInfo pci(boost::dynamic_pointer_cast<EnvironmentBase>(gEnv->shared_from_this()));
  bool construct_success = pci.fromJson(root);
  ASSERT_TRUE(construct_success);
  TrajOptProbPtr prob = ConstructProblem(pci);

  ASSERT_TRUE(!!prob);
  BasicTrustRegionSQP opt(prob);
//  opt.addCallback(boost::bind(&PlotStateAndIdle, boost::ref(rad), _1));
  opt.addCallback(boost::bind(&PlotCosts, boost::ref(prob->getCosts()),boost::ref(rad),  _1));
  opt.initialize(DblVec(prob->GetNumDOF(), 0));
  opt.optimize();
  rad.SetDOFValues(opt.x());
}

int main(int argc, char** argv)
{

  srand(0);



  ::testing::InitGoogleTest(&argc, argv);

  RaveInitialize(true);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  env->Load("/Users/joschu/Desktop/puma.robot.xml");
  gEnv = env.get();

  OSGViewerPtr viewer(new OSGViewer(env));
  env->AddViewer(viewer);
  gViewer = viewer.get();
  RUN_ALL_TESTS();
//  viewer.reset();
  RaveDestroy();
}

//
//TEST(arm_planning, straight_line_in_joint_space) {
//
//
//}
//
//TEST(arm_planning, up_constraint) {
//
//}
//
//TEST(arm_planning, pr2_arm_around_table) {
//
//}

//TrajOptProblemPtr CreateProblemFromRequest
