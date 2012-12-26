#include <gtest/gtest.h>
#include <openrave/openrave.h>
#include "trajopt/collision_checker.hpp"
#include "utils/general_utils.hpp"
#include "trajopt/common.hpp"
#include "trajopt/problem_description.hpp"
using namespace trajopt;
using namespace std;
using Eigen::Vector3d;

namespace {
void getVec(Json::Value& v, const Vector3d& x) {
  for (int i=0; i < 3; ++i) v.append(x[i]);
}
bool appendVec(Json::Value& v, const Vector3d& x) {
  for (int i=0; i < 3; ++i) v.append(x[i]);
  return true;
}

}


TEST(arm_planning, straight_line_in_joint_space) {


}

TEST(arm_planning, up_constraint) {

}

TEST(arm_planning, pr2_arm_around_table) {

}

//TrajOptProblemPtr CreateProblemFromRequest
