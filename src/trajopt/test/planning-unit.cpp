#include <gtest/gtest.h>
#include <openrave-core.h>
#include <openrave/openrave.h>
#include "trajopt/collision_checker.hpp"
#include "utils/stl_to_string.hpp"
#include "trajopt/common.hpp"
#include "trajopt/problem_description.hpp"
#include "ipi/sco/optimizers.hpp"
#include "trajopt/rave_utils.hpp"
#include "osgviewer/osgviewer.hpp"
#include <ctime>
#include "utils/eigen_conversions.hpp"
#include "utils/clock.hpp"
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include "utils/config.hpp"
#include "trajopt/plot_callback.hpp"
using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;

namespace {

bool plotting=false, verbose=false;


OR::Transform randomReachablePose(RobotAndDOF& rad, KinBody::LinkPtr link) {
  // todo: save & restore
  DblVec dofvals = rad.RandomDOFValues();
  rad.SetDOFValues(dofvals);
  return link->GetTransform();
}

template<typename T>
void toJson(Json::Value& v, const T& x) {
  v.resize(x.size());
  for (int i=0; i < x.size(); ++i) {
    v[i] = x[i];
  }
}


Json::Value readJsonFile(const std::string& fname) {
  Json::Value root;
  Json::Reader reader;
  ifstream fh(fname.c_str());
  bool parse_success = reader.parse(fh, root);
  if (!parse_success) throw std::runtime_error("failed to parse " +fname);
  return root;
}


}



//RobotBase::ManipulatorPtr GetOnlyManipulator(RobotBase& robot) {
//  vector<RobotBase::ManipulatorPtr> manips = robot.GetManipulators();
//  assert(manips.size()==1);
//  return manips[0];
//}




class PlanningTest : public testing::TestWithParam<const char*> {
public:
  static EnvironmentBasePtr env;
  static OSGViewerPtr viewer;

  void SetUp() {
    RobotBasePtr robot = GetRobot(*env);
    robot->SetDOFValues(DblVec(robot->GetDOF(), 0));
    Transform I; I.identity();
    robot->SetTransform(I);
  }


  static void SetUpTestCase() {
    RaveInitialize(true, verbose ? Level_Debug : Level_Info);
    env = RaveCreateEnvironment();
    env->StopSimulation();
    env->Load("robots/pr2-beta-static.zae") ;
    env->Load(string(DATA_DIR) + "/table.xml");
    viewer.reset(new OSGViewer(env));
    viewer->UpdateSceneData();
    env->AddViewer(viewer);
  }

  static void TearDownTestCase() {
    viewer.reset();
    env.reset();

    RaveDestroy();

  }
};
EnvironmentBasePtr PlanningTest::env;
OSGViewerPtr PlanningTest::viewer;

TEST_F(PlanningTest, numerical_ik1) {
  Json::Value root = readJsonFile(string(DATA_DIR) + "/numerical_ik1.json");
  TrajOptProbPtr prob = ConstructProblem(root, env);
  ASSERT_TRUE(!!prob);

  BasicTrustRegionSQP opt(prob);
//  opt.addCallback(boost::bind(&PlotCosts, boost::ref(prob->getCosts()),*prob->GetRAD(), prob->GetVars(), _1));
  opt.initialize(DblVec(prob->GetNumDOF(), 0));
  double tStart = GetClock();
  opt.optimize();
  RAVELOG_INFO("planning time: %.3f\n", GetClock()-tStart);
  printf("finally:\n");
  PlotCallback(*prob)(opt.x());
}

TEST_F(PlanningTest, arm_around_table) {
  RobotBasePtr pr2 = GetRobot(*env);

  ProblemConstructionInfo pci(env);
  Json::Value root = readJsonFile(string(DATA_DIR) + "/arm_around_table.json");
  pci.fromJson(root);
  pci.rad->SetDOFValues(toDblVec(pci.init_info.data.row(0)));
  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);


  BasicTrustRegionSQP opt(prob);
  if (plotting) opt.addCallback(PlotCallback(*prob));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();
  opt.optimize();
  RAVELOG_INFO("planning time: %.3f\n", GetClock()-tStart);

  PlotCallback(*prob)(opt.x());
}


int main(int argc, char** argv)
{
  {
    Config config;
    config.add(new Parameter<bool>("plotting", &plotting, "plotting"));
    config.add(new Parameter<bool>("verbose", &verbose, "verbose"));
    CommandParser parser(config);
    parser.read(argc, argv);
  }
  srand(0);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
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
