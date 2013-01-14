#include <boost/python.hpp>
#include "trajopt/collision_checker.hpp"
#include "trajopt/problem_description.hpp"
#include <stdexcept>
#include <boost/python/exception_translator.hpp>
#include <boost/foreach.hpp>

using namespace trajopt;
using namespace Eigen;
using namespace OpenRAVE;
using std::vector;

namespace py = boost::python;

namespace {
bool gInteractive = true;

}

class PyTrajOptProb {
public:
  TrajOptProbPtr m_prob;
  PyTrajOptProb(TrajOptProbPtr prob) : m_prob(prob) {}
};

Json::Value readJsonFile(const std::string& doc) {
  Json::Value root;
  Json::Reader reader;
  bool success = reader.parse(doc, root);
  if (!success) throw openrave_exception("couldn't parse string as json");
  return root;
}

PyTrajOptProb PyConstructProblem(const std::string& json_string, py::object py_env) {
  py::object openravepy = py::import("openravepy");
  int id = py::extract<int>(openravepy.attr("RaveGetEnvironmentId")(py_env));
  EnvironmentBasePtr cpp_env = RaveGetEnvironment(id);
  Json::Value json_root = readJsonFile(json_string);
  TrajOptProbPtr cpp_prob = ConstructProblem(json_root, cpp_env);
  return PyTrajOptProb(cpp_prob);
}

void SetInteractive(py::object b) {
  gInteractive = py::extract<bool>(b);
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
  py::object GetTraj() {
    py::object numpy = py::import("numpy");
    TrajArray &traj = m_result->traj;
    py::object out = numpy.attr("empty")(py::make_tuple(traj.rows(), traj.cols()));
    for (int i = 0; i < traj.rows(); ++i) {
      for (int j = 0; j < traj.cols(); ++j) {
        out[i][j] = traj(i, j);
      }
    }
    return out;
  }
  py::object __str__() {
    return GetCosts().attr("__str__")() + GetConstraints().attr("__str__")();
  }
};

PyTrajOptResult PyOptimizeProblem(PyTrajOptProb& prob) {
  return OptimizeProblem(prob.m_prob, gInteractive);
}


class PyCollision {
public:
  Collision m_c;
  PyCollision(const Collision& c) : m_c(c) {}
  float GetDistance() {return m_c.distance;}
};

py::list toPyList(const vector<Collision>& collisions) {
  py::list out;
  BOOST_FOREACH(const Collision& c, collisions) {
    out.append(PyCollision(c));
  }
  return out;
}

class PyCollisionChecker {
public:
  py::object AllVsAll() {
    vector<Collision> collisions;
    m_cc->AllVsAll(collisions);
    return toPyList(collisions);
  }
  py::object BodyVsAll(py::object py_kb) {
    KinBodyPtr cpp_kb = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv())
        ->GetBodyFromEnvironmentId(py::extract<int>(py_kb.attr("GetEnvironmentId")()));
    if (!cpp_kb) {
      throw openrave_exception("body isn't part of environment!");
    }
    vector<Collision> collisions;
    m_cc->BodyVsAll(*cpp_kb, collisions);
    return toPyList(collisions);
  }
  PyCollisionChecker(CollisionCheckerPtr cc) : m_cc(cc) {}
private:
  PyCollisionChecker();
  CollisionCheckerPtr m_cc;
};


PyCollisionChecker PyGetCollisionChecker(py::object py_env) {
  py::object openravepy = py::import("openravepy");
  int id = py::extract<int>(openravepy.attr("RaveGetEnvironmentId")(py_env));
  EnvironmentBasePtr cpp_env = RaveGetEnvironment(id);
  CollisionCheckerPtr cc = CollisionChecker::GetOrCreate(*cpp_env);
  return PyCollisionChecker(cc);
}

BOOST_PYTHON_MODULE(ctrajoptpy) {

  py::class_<PyTrajOptProb>("TrajOptProb", py::no_init)
  ;
  py::def("SetInteractive", &SetInteractive);
  py::def("ConstructProblem", &PyConstructProblem);
  py::def("OptimizeProblem", &PyOptimizeProblem);

  py::class_<PyTrajOptResult>("TrajOptResult", py::no_init)
      .def("GetCosts", &PyTrajOptResult::GetCosts)
      .def("GetConstraints", &PyTrajOptResult::GetConstraints)
      .def("GetTraj", &PyTrajOptResult::GetTraj)
      .def("__str__", &PyTrajOptResult::__str__)
      ;

  py::class_<PyCollisionChecker>("CollisionChecker", py::no_init)
      .def("AllVsAll", &PyCollisionChecker::AllVsAll)
      .def("BodyVsAll", &PyCollisionChecker::BodyVsAll)
      ;
  py::def("GetCollisionChecker", &PyGetCollisionChecker);
  py::class_<PyCollision>("Collision", py::no_init)
     .def("GetDistance", &PyCollision::GetDistance)
    ;

}
