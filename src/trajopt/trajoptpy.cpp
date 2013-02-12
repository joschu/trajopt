#include <boost/python.hpp>
#include "trajopt/collision_checker.hpp"
#include "trajopt/problem_description.hpp"
#include "osgviewer/osgviewer.hpp"
#include <boost/foreach.hpp>

using namespace trajopt;
using namespace Eigen;
using namespace OpenRAVE;
using std::vector;

namespace py = boost::python;

namespace {
bool gInteractive = true;
py::object openravepy;

py::list toPyList(const IntVec& x) {
  py::list out;
  for (int i=0; i < x.size(); ++i) out.append(x[i]);
  return out;
}


EnvironmentBasePtr GetCppEnv(py::object py_env) {
  int id = py::extract<int>(openravepy.attr("RaveGetEnvironmentId")(py_env));
  EnvironmentBasePtr cpp_env = RaveGetEnvironment(id);
  return cpp_env;
}
KinBodyPtr GetCppKinBody(py::object py_kb, EnvironmentBasePtr env) {
  int id = py::extract<int>(py_kb.attr("GetEnvironmentId")());
  return env->GetBodyFromEnvironmentId(id);
}
KinBody::LinkPtr GetCppLink(py::object py_link, EnvironmentBasePtr env) {
  KinBodyPtr parent = GetCppKinBody(py_link.attr("GetParent")(), env);
  int idx = py::extract<int>(py_link.attr("GetIndex")());
  return parent->GetLinks()[idx];
}


}

class PyTrajOptProb {
public:
  TrajOptProbPtr m_prob;
  PyTrajOptProb(TrajOptProbPtr prob) : m_prob(prob) {}
  py::list GetDOFIndices() {
    vector<int> inds = m_prob->GetRAD()->GetJointIndices();
    return toPyList(inds);
  }
};

Json::Value readJsonFile(const std::string& doc) {
  Json::Value root;
  Json::Reader reader;
  bool success = reader.parse(doc, root);
  if (!success) throw openrave_exception("couldn't parse string as json");
  return root;
}

PyTrajOptProb PyConstructProblem(const std::string& json_string, py::object py_env) {
  EnvironmentBasePtr cpp_env = GetCppEnv(py_env);
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

class PyGraphHandle {
  vector<GraphHandlePtr> m_handles;
public:
  PyGraphHandle(const vector<GraphHandlePtr>& handles) : m_handles(handles) {}
  PyGraphHandle(GraphHandlePtr handle) : m_handles(1, handle) {}
  void SetTransparency1(float alpha) {
    BOOST_FOREACH(GraphHandlePtr& handle, m_handles) {
      SetTransparency(handle, alpha);
    }
  }
};

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
  PyGraphHandle PlotCollisionGeometry() {
    vector<GraphHandlePtr> handles;
    m_cc->PlotCollisionGeometry(handles);
    return PyGraphHandle(handles);
  }
  void ExcludeCollisionPair(py::object link0, py::object link1) {
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());
    m_cc->ExcludeCollisionPair(*GetCppLink(link0, env), *GetCppLink(link1, env));
  }
  PyCollisionChecker(CollisionCheckerPtr cc) : m_cc(cc) {}
private:
  PyCollisionChecker();
  CollisionCheckerPtr m_cc;
};



PyCollisionChecker PyGetCollisionChecker(py::object py_env) {
  CollisionCheckerPtr cc = CollisionChecker::GetOrCreate(*GetCppEnv(py_env));
  return PyCollisionChecker(cc);
}

class PyOSGViewer {
  OSGViewerPtr m_viewer;
public:
    PyOSGViewer(OSGViewerPtr viewer) : m_viewer(viewer) {}
  int Step() {
    m_viewer->UpdateSceneData();
    m_viewer->Draw();
    return 0;
  }
  PyGraphHandle PlotKinBody(py::object py_kb) {
    return PyGraphHandle(m_viewer->PlotKinBody(GetCppKinBody(py_kb, m_viewer->GetEnv())));
  }
  void SetAllTransparency(float a) {
    m_viewer->SetAllTransparency(a);
  }
  void Idle() {
    m_viewer->Idle();
  }
};
PyOSGViewer PyGetViewer(py::object py_env) {
  EnvironmentBasePtr env = GetCppEnv(py_env);
  ViewerBasePtr viewer = OSGViewer::GetOrCreate(env);
  return PyOSGViewer(boost::dynamic_pointer_cast<OSGViewer>(viewer));
}


BOOST_PYTHON_MODULE(ctrajoptpy) {

  openravepy = py::import("openravepy");

  py::class_<PyTrajOptProb>("TrajOptProb", py::no_init)
      .def("GetDOFIndices", &PyTrajOptProb::GetDOFIndices)
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
      .def("PlotCollisionGeometry", &PyCollisionChecker::PlotCollisionGeometry)
      .def("ExcludeCollisionPair", &PyCollisionChecker::ExcludeCollisionPair)
      ;
  py::def("GetCollisionChecker", &PyGetCollisionChecker);
  py::class_<PyCollision>("Collision", py::no_init)
     .def("GetDistance", &PyCollision::GetDistance)
    ;
  py::class_< PyGraphHandle >("GraphHandle", py::no_init)
     .def("SetTransparency", &PyGraphHandle::SetTransparency1)
     ;

  py::class_< PyOSGViewer >("OSGViewer", py::no_init)
     .def("Step", &PyOSGViewer::Step)
     .def("PlotKinBody", &PyOSGViewer::PlotKinBody)
     .def("SetAllTransparency", &PyOSGViewer::SetAllTransparency)
     .def("Idle", &PyOSGViewer::Idle)
    ;
  py::def("GetViewer", &PyGetViewer);

}
