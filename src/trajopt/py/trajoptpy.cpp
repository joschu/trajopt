#include <boost/python.hpp>
#include "trajopt/collision_checker.hpp"
#include "trajopt/problem_description.hpp"
#include <stdexcept>
#include <boost/python/exception_translator.hpp>

using namespace trajopt;
using namespace Eigen;
using namespace OpenRAVE;

namespace py = boost::python;

namespace {
bool gInteractive = true;

}

class PyTrajOptProb {
public:
  TrajOptProbPtr m_prob;
  PyTrajOptProb(TrajOptProbPtr prob) : m_prob(prob) {}
};
typedef boost::shared_ptr<PyTrajOptProb> PyTrajOptProbPtr;

Json::Value readJsonFile(const std::string& doc) {
  Json::Value root;
  Json::Reader reader;
  bool success = reader.parse(doc, root);
  if (!success) throw openrave_exception("couldn't parse string as json");
  return root;
}

PyTrajOptProbPtr PyConstructProblem(const std::string& json_string, py::object py_env) {
  py::object openravepy = py::import("openravepy");
  int id = py::extract<int>(openravepy.attr("RaveGetEnvironmentId")(py_env));
  EnvironmentBasePtr cpp_env = RaveGetEnvironment(id);
  Json::Value json_root = readJsonFile(json_string);
  TrajOptProbPtr cpp_prob = ConstructProblem(json_root, cpp_env);
  return PyTrajOptProbPtr(new PyTrajOptProb(cpp_prob));
}

void SetInteractive(bool b) {
  gInteractive = b;
}

class PyTrajOptResult {
public:
  PyTrajOptResult(TrajOptResultPtr result) : m_result(result) {}
  TrajOptResultPtr m_result;
  py::object GetCosts() {
    py::list out;
    int n_costs = m_result->cost_names.size();
    for (int i=0; i < n_costs; ++i) {
      out.append(py::make_tuple(m_result->cost_names[i], m_result->cost_vals[i]));
    }
    return out;
  }
  py::object GetConstraints() {
    py::list out;
    int n_cnts = m_result->cnt_names.size();
    for (int i=0; i < n_cnts; ++i) {
      out.append(py::make_tuple(m_result->cnt_names[i], m_result->cnt_viols[i]));
    }
    return out;
  }
  py::object __str__() {
    return GetCosts().attr("__str__")() + GetConstraints().attr("__str__")();
  }
};

PyTrajOptResult PyOptimizeProblem(PyTrajOptProbPtr prob) {
  return OptimizeProblem(prob->m_prob, gInteractive);
}

BOOST_PYTHON_MODULE(trajoptpy) {

//  py::class_<std::runtime_error>("RuntimeError", py::init<std::string>())
//     .def("__str__", &std::runtime_error::what)
//     ;

  py::class_<PyTrajOptProb,PyTrajOptProbPtr, boost::noncopyable>("TrajOptProb", py::no_init)
  ;
  py::def("SetInteractive", &SetInteractive);
  py::def("ConstructProblem", &PyConstructProblem);
  py::def("OptimizeProblem", &PyOptimizeProblem);

  py::class_<PyTrajOptResult>("TrajOptResult", py::no_init)
      .def("GetCosts", &PyTrajOptResult::GetCosts)
      .def("GetConstraints", &PyTrajOptResult::GetConstraints)
      .def("__str__", &PyTrajOptResult::__str__)
      ;


  ;
}
