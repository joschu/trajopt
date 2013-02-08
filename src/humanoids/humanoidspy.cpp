#include <boost/python.hpp>
namespace py = boost::python;
namespace trajopt{
  extern void RegisterHumanoidCostsAndCnts();
}

BOOST_PYTHON_MODULE(humanoidspy) {
  trajopt::RegisterHumanoidCostsAndCnts();
}

