#pragma once
#include <vector>
#include <map>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>

#include "ipi/sco/modeling.hpp"
#include "utils/basic_array.hpp"
#include "trajopt/macros.h"

namespace trajopt {


namespace OR = OpenRAVE;
using OR::KinBody;
using OR::RobotBase;
using std::vector;
using std::map;
using namespace ipi::sco;
using namespace util;

typedef BasicArray<Var> VarArray;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;

typedef vector<double> DblVec;
typedef vector<int> IntVec;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;


class Plotter {
public:
  virtual void Plot(const DblVec& x, OR::EnvironmentBase&, std::vector<OR::GraphHandlePtr>& handles) = 0;
  virtual ~Plotter() {}
};

}
