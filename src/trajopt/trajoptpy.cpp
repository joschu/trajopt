#include <boost/python.hpp>
#include "trajopt/collision_checker.hpp"
#include "trajopt/problem_description.hpp"
#include "osgviewer/osgviewer.hpp"
#include "osgviewer/robot_ui.hpp"
#include <boost/foreach.hpp>
#include "macros.h"
#include "sco/modeling_utils.hpp"
#include "openrave_userdata_utils.hpp"
#include "numpy_utils.hpp"
#include <limits>
#include "utils/eigen_conversions.hpp"
#include "trajopt/rave_utils.hpp"
using namespace trajopt;
using namespace Eigen;
using namespace OpenRAVE;
using std::vector;

namespace py = boost::python;

bool gInteractive = false;


EnvironmentBasePtr GetCppEnv(py::object py_env) {
  py::object openravepy = py::import("openravepy");
  int id = py::extract<int>(openravepy.attr("RaveGetEnvironmentId")(py_env));
  EnvironmentBasePtr cpp_env = RaveGetEnvironment(id);
  return cpp_env;
}
KinBodyPtr GetCppKinBody(py::object py_kb, EnvironmentBasePtr env) {
  KinBodyPtr cpp_kb;
  if (PyObject_HasAttrString(py_kb.ptr(), "GetEnvironmentId")) {
    int id = py::extract<int>(py_kb.attr("GetEnvironmentId")());
    cpp_kb = env->GetBodyFromEnvironmentId(id);
  }
  return cpp_kb;
}
KinBody::LinkPtr GetCppLink(py::object py_link, EnvironmentBasePtr env) {
  KinBody::LinkPtr cpp_link;
  if (PyObject_HasAttrString(py_link.ptr(), "GetParent")) {
    KinBodyPtr parent = GetCppKinBody(py_link.attr("GetParent")(), env);
    int idx = py::extract<int>(py_link.attr("GetIndex")());
    cpp_link = parent->GetLinks()[idx];
  }
  return cpp_link;
}
RobotBasePtr GetCppRobot(py::object py_robot, EnvironmentBasePtr env) {
  return boost::dynamic_pointer_cast<RobotBase>(GetCppKinBody(py_robot, env));
}
RobotBase::ManipulatorPtr GetCppManip(py::object py_manip, EnvironmentBasePtr env) {
  RobotBase::ManipulatorPtr cpp_manip;
  if (PyObject_HasAttrString(py_manip.ptr(), "GetRobot")) {
    RobotBasePtr robot = GetCppRobot(py_manip.attr("GetRobot")(), env);
    cpp_manip = robot->GetManipulator(py::extract<string>(py_manip.attr("GetName")()));
  }
  return cpp_manip;
}
vector<KinBody::LinkPtr> GetCppLinks(py::object py_obj, EnvironmentBasePtr env) {
  vector<KinBody::LinkPtr> links;
  KinBodyPtr cpp_kb = GetCppKinBody(py_obj, env);
  if (!!cpp_kb) links.insert(links.end(), cpp_kb->GetLinks().begin(), cpp_kb->GetLinks().end());
  KinBody::LinkPtr cpp_link = GetCppLink(py_obj, env);
  if (!!cpp_link) links.push_back(cpp_link);
  return links;
}


class PyTrajOptProb {
public:
  TrajOptProbPtr m_prob;
  PyTrajOptProb(TrajOptProbPtr prob) : m_prob(prob) {}
  py::list GetDOFIndices() {
    RobotAndDOFPtr rad = boost::dynamic_pointer_cast<RobotAndDOF>(m_prob->GetRAD());
    if (!rad) PRINT_AND_THROW("can only call GetDOFIndices on a robot");
    vector<int> inds = rad->GetJointIndices();
    return toPyList(inds);
  }
  void SetRobotActiveDOFs() {
    RobotAndDOFPtr rad = boost::dynamic_pointer_cast<RobotAndDOF>(m_prob->GetRAD());
    if (!rad) PRINT_AND_THROW("can only call SetRobotActiveDOFs on a robot");
    rad->SetRobotActiveDOFs();
  }
  void AddConstraint1(py::object f, py::list ijs, const string& typestr, const string& name);
  void AddConstraint2(py::object f, py::object dfdx, py::list ijs, const string& typestr, const string& name);
  void AddCost1(py::object f, py::list ijs, const string& name);
  void AddErrCost1(py::object f, py::list ijs, const string& typestr, const string& name);
  void AddErrCost2(py::object f, py::object dfdx, py::list ijs, const string& typestr, const string& name);
};

struct ScalarFuncFromPy : public ScalarOfVector {
  py::object m_pyfunc;
  ScalarFuncFromPy(py::object pyfunc) : m_pyfunc(pyfunc) {}
  double operator()(const VectorXd& x) const {
    return py::extract<double>(m_pyfunc(toNdarray1<double>(x.data(), x.size())));
  }
};
struct VectorFuncFromPy : public VectorOfVector {
  py::object m_pyfunc;
  VectorFuncFromPy(py::object pyfunc) : m_pyfunc(pyfunc) {}
  VectorXd operator()(const VectorXd& x) const {
    py::object outarr = np_mod.attr("array")(m_pyfunc(toNdarray1<double>(x.data(), x.size())), "float64");
    VectorXd out = Map<const VectorXd>(getPointer<double>(outarr), py::extract<int>(outarr.attr("size")));
    return out;
  }
};
struct MatrixFuncFromPy : public MatrixOfVector {
  py::object m_pyfunc;
  MatrixFuncFromPy(py::object pyfunc) : m_pyfunc(pyfunc) {}
  MatrixXd operator()(const VectorXd& x) const {
    py::object outarr = np_mod.attr("array")(m_pyfunc(toNdarray1<double>(x.data(), x.size())),"float64");
    py::object shape = outarr.attr("shape");
    MatrixXd out = Map<const MatrixXd>(getPointer<double>(outarr), py::extract<int>(shape[0]), py::extract<int>(shape[1]));
    return out;
  }
};

ConstraintType _GetConstraintType(const string& typestr) {
  if (typestr == "EQ") return EQ;
  else if (typestr == "INEQ") return INEQ;
  else PRINT_AND_THROW("type must be \"EQ\" or \"INEQ\"");  
}
PenaltyType _GetPenaltyType(const string& typestr) {
  if (typestr == "SQUARED") return SQUARED;
  else if (typestr == "ABS") return ABS;
  else if (typestr == "HINGE") return HINGE;
  else PRINT_AND_THROW("type must be \"SQUARED\" or \"ABS\" or \"HINGE\"r");  
}
VarVector _GetVars(py::list ijs, const VarArray& vars) {
  VarVector out;
  int n = py::len(ijs);
  for (int k=0; k < n; ++k) {
    int i = py::extract<int>(ijs[k][0]);
    int j = py::extract<int>(ijs[k][1]);
    out.push_back(vars(i,j));
  }  
  return out;
}

void PyTrajOptProb::AddConstraint1(py::object f, py::list ijs, const string& typestr, const string& name) {  
  ConstraintType type = _GetConstraintType(typestr);
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  ConstraintPtr c(new ConstraintFromFunc(VectorOfVectorPtr(new VectorFuncFromPy(f)), vars, VectorXd::Ones(0), type, name));
  m_prob->addConstraint(c);
}
void PyTrajOptProb::AddConstraint2(py::object f, py::object dfdx, py::list ijs, const string& typestr, const string& name) {
  ConstraintType type = _GetConstraintType(typestr);
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  ConstraintPtr c(new ConstraintFromFunc(VectorOfVectorPtr(new VectorFuncFromPy(f)), MatrixOfVectorPtr(new MatrixFuncFromPy(dfdx)), vars, VectorXd::Ones(0), type, name));
  m_prob->addConstraint(c);
}
void PyTrajOptProb::AddCost1(py::object f, py::list ijs, const string& name) {
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  CostPtr c(new CostFromFunc(ScalarOfVectorPtr(new ScalarFuncFromPy(f)), vars, "f"));
  m_prob->addCost(c);
}
void PyTrajOptProb::AddErrCost1(py::object f, py::list ijs, const string& typestr, const string& name) {
  PenaltyType type = _GetPenaltyType(typestr);
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  CostPtr c(new CostFromErrFunc(VectorOfVectorPtr(new VectorFuncFromPy(f)), vars, VectorXd(), type, name));
  m_prob->addCost(c);
}
void PyTrajOptProb::AddErrCost2(py::object f, py::object dfdx, py::list ijs, const string& typestr, const string& name) {
  PenaltyType type = _GetPenaltyType(typestr);
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  CostPtr c(new CostFromErrFunc(VectorOfVectorPtr(new VectorFuncFromPy(f)), MatrixOfVectorPtr(new MatrixFuncFromPy(dfdx)), vars, VectorXd(), type, name));
  m_prob->addCost(c);
}


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

class PyCollision {
public:
  Collision m_c;
  PyCollision(const Collision& c) : m_c(c) {}
  float GetDistance() {return m_c.distance;}
  py::object GetNormal() {return toNdarray1<double>((double*)&m_c.normalB2A,3);}
  py::object GetPtA() {return toNdarray1<double>((double*)&m_c.ptA,3);}
  py::object GetPtB() {return toNdarray1<double>((double*)&m_c.ptB,3);}
  string GetLinkAName() {return m_c.linkA->GetName();}
  string GetLinkBName() {return m_c.linkB->GetName();}
  string GetLinkAParentName() {return m_c.linkA->GetParent()->GetName();}
  string GetLinkBParentName() {return m_c.linkB->GetParent()->GetName();}
  py::object GetMultiCastAlphas() {return toNdarray1<float>(&m_c.mi.alpha[0], m_c.mi.alpha.size());}
  py::object GetMultiCastIndices() {return toNdarray1<int>(&m_c.mi.instance_ind[0], m_c.mi.instance_ind.size());}
};

py::list toPyList(const vector<Collision>& collisions) {
  py::list out;
  BOOST_FOREACH(const Collision& c, collisions) {
    out.append(PyCollision(c));
  }
  return out;
}

bool compareCollisions(const Collision& c1,const Collision& c2) { return c1.distance < c2.distance; }

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
    TrajArray &traj = m_result->traj;    
    return toNdarray2<double>(traj.data(),traj.rows(), traj.cols());    
  }
  py::object GetCollisions() {
    std::sort(m_result->colsContinuous.begin(), m_result->colsContinuous.end(), compareCollisions);
    return toPyList(m_result->colsContinuous);
  }
  py::object __str__() {
    return GetCosts().attr("__str__")() + GetConstraints().attr("__str__")();
  }
};

PyTrajOptResult PyOptimizeProblem(PyTrajOptProb& prob) {
  return OptimizeProblem(prob.m_prob, gInteractive);
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
  py::object BodyVsAll(py::object& py_kb, bool sort=true) {
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());
    KinBodyPtr cpp_kb = GetCppKinBody(py_kb, env);
    if (!cpp_kb) {
      throw openrave_exception("body isn't part of environment!");
    }
    vector<Collision> collisions;
    m_cc->BodyVsAll(*cpp_kb, collisions);
    if (sort)
      std::sort(collisions.begin(), collisions.end(), compareCollisions);
    return toPyList(collisions);
  }
  py::object BodyVsBody(py::object& py_kb1, py::object& py_kb2, bool sort=true) {
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());
    KinBodyPtr cpp_kb1 = GetCppKinBody(py_kb1, env);
    KinBodyPtr cpp_kb2 = GetCppKinBody(py_kb2, env);
    if (!cpp_kb1) {
      throw openrave_exception("body 1 isn't part of environment!");
    }
    if (!cpp_kb2) {
      throw openrave_exception("body 2 isn't part of environment!");
    }
    vector<Collision> collisions;
    m_cc->BodyVsBody(*cpp_kb1, *cpp_kb2, collisions);
    if (sort)
      std::sort(collisions.begin(), collisions.end(), compareCollisions);
    return toPyList(collisions);
  }
  py::object BodiesVsBodies(py::list& py_kbs1, py::list& py_kbs2) {
    //py::list py_kbs vs np::list py_kbs
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());

    //Convert Bodies
    int n_kbs1 = py::extract<int>(py_kbs1.attr("__len__")());
    vector<KinBodyPtr> cpp_kbs1(n_kbs1);
    for (int i=0; i < n_kbs1; ++i) {
      cpp_kbs1[i] = GetCppKinBody(py_kbs1[i], env);
      if (!cpp_kbs1[i]) {
        throw openrave_exception("One of the bodies isn't part of environment!");
      }
    }

    int n_kbs2 = py::extract<int>(py_kbs2.attr("__len__")());
    vector<KinBodyPtr> cpp_kbs2(n_kbs2);
    for (int i=0; i < n_kbs2; ++i) {
      cpp_kbs2[i] = GetCppKinBody(py_kbs2[i], env);
      if (!cpp_kbs2[i]) {
        throw openrave_exception("One of the bodies isn't part of environment!");
      }
    }

    vector< vector<Collision> > collisions(n_kbs1);
    for (int i=0; i < n_kbs1; ++i) { 
      for (int j=0; j < n_kbs2; ++j) {
        m_cc->BodyVsBody(*cpp_kbs1[i], *cpp_kbs2[j], collisions[i]);
      }
    }

    vector<Collision> f_collisions;
    for (int i=0; i < n_kbs1; ++i) {
      std::sort(collisions[i].begin(), collisions[i].end(), compareCollisions);
      if (!collisions[i].empty()) {
        f_collisions.push_back(collisions[i].front());
      }
    }
    
    return toPyList(f_collisions);
  }
  py::object CastVsAll(py::object& py_kb, py::object& py_dv0, py::object& py_dv1, string which_dofs="active", bool sort=true) {
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());
    KinBodyPtr cpp_kb = GetCppKinBody(py_kb, env);
    if (!cpp_kb) {
      throw openrave_exception("Kinbody isn't part of environment!");
    }
    RobotBasePtr robot = boost::dynamic_pointer_cast<RobotBase>(cpp_kb);
    if (!robot) {
      throw openrave_exception("Kinbody isn't a robot!");
    }

    int n_dofs0 = py::extract<int>(py_dv0.attr("__len__")());
    DblVec dofvals0(n_dofs0);
    for (unsigned i=0; i < n_dofs0; ++i) dofvals0[i] = py::extract<double>(py_dv0[i]);

    int n_dofs1 = py::extract<int>(py_dv1.attr("__len__")());
    DblVec dofvals1(n_dofs1);
    for (unsigned i=0; i < n_dofs1; ++i) dofvals1[i] = py::extract<double>(py_dv1[i]);

    ConfigurationPtr rad = RADFromName(which_dofs, robot);
    vector<int> inds;
    std::vector<KinBody::LinkPtr> links;
    rad->GetAffectedLinks(links,true,inds);

    vector<Collision> collisions;
    m_cc->CastVsAll(*rad, links, dofvals0, dofvals1, collisions);
    if (sort)
      std::sort(collisions.begin(), collisions.end(), compareCollisions);
    return toPyList(collisions);
  }
  py::object MultiCastVsAll(py::object& py_kb, py::object& py_dof_list, string which_dofs="active", bool prevent_z=true, bool sort=true) {
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());
    KinBodyPtr cpp_kb = GetCppKinBody(py_kb, env);
    if (!cpp_kb) {
      throw openrave_exception("Kinbody isn't part of environment!");
    }
    RobotBasePtr robot = boost::dynamic_pointer_cast<RobotBase>(cpp_kb);
    if (!robot) {
      throw openrave_exception("Kinbody isn't a robot!");
    }

    int n_elements = py::extract<int>(py_dof_list.attr("__len__")());
    int n_dofs = 0;
    if (n_elements > 0)
      n_dofs = py::extract<int>(py_dof_list[0].attr("__len__")());
    vector<DblVec> dofvals(n_elements, DblVec(n_dofs));
    for (int i=0; i<n_elements; i++) {
      for (int j=0; j<n_dofs; j++) {
        dofvals[i][j] = py::extract<double>(py_dof_list[i][j]);
      }
    }

    ConfigurationPtr rad = RADFromName(which_dofs, robot);
    vector<int> inds;
    std::vector<KinBody::LinkPtr> links;
    rad->GetAffectedLinks(links,true,inds);
    vector<Collision> collisions;
    m_cc->MultiCastVsAll(*rad, links, dofvals, collisions, (robot->GetAffineDOF() > 0) && prevent_z );
    if (sort)
      std::sort(collisions.begin(), collisions.end(), compareCollisions);
    return toPyList(collisions);
  }
  void SetContactDistance(float dist) {
    m_cc->SetContactDistance(dist);
  }
  double GetContactDistance() {
    return m_cc->GetContactDistance();
  }
  PyGraphHandle PlotCollisionGeometry() {
    vector<GraphHandlePtr> handles;
    m_cc->PlotCollisionGeometry(handles);
    return PyGraphHandle(handles);
  }
  void ExcludeCollisionPair(py::object py_obj0, py::object py_obj1) {
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());
    
    vector<KinBody::LinkPtr> links0 = GetCppLinks(py_obj0, env);
    vector<KinBody::LinkPtr> links1 = GetCppLinks(py_obj1, env);

    BOOST_FOREACH(const KinBody::LinkPtr& link0, links0) {
      BOOST_FOREACH(const KinBody::LinkPtr& link1, links1) {
        m_cc->ExcludeCollisionPair(*link0, *link1);
      }
    }
  }
  void IncludeCollisionPair(py::object py_obj0, py::object py_obj1) {
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());
    
    vector<KinBody::LinkPtr> links0 = GetCppLinks(py_obj0, env);
    vector<KinBody::LinkPtr> links1 = GetCppLinks(py_obj1, env);

    BOOST_FOREACH(const KinBody::LinkPtr& link0, links0) {
      BOOST_FOREACH(const KinBody::LinkPtr& link1, links1) {
        m_cc->IncludeCollisionPair(*link0, *link1);
      }
    }
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


void CallPyFunc(py::object f) {
  f();
}

class PyOSGViewer {
public:
    PyOSGViewer(OSGViewerPtr viewer) : m_viewer(viewer) {}
  int Step() {
    m_viewer->Draw();
    return 0;
  }
  void UpdateSceneData() {
    m_viewer->UpdateSceneData();
  }
  PyGraphHandle PlotKinBody(py::object py_kb) {
    return PyGraphHandle(m_viewer->PlotKinBody(GetCppKinBody(py_kb, m_viewer->GetEnv())));
  }
  PyGraphHandle PlotLink(py::object py_link) {
    return PyGraphHandle(m_viewer->PlotLink(GetCppLink(py_link, m_viewer->GetEnv())));
  }
  void SetTransparency(py::object py_kb, float alpha) {
    m_viewer->SetTransparency(GetCppKinBody(py_kb, m_viewer->GetEnv()), alpha);
  }
  void SetAllTransparency(float a) {
    m_viewer->SetAllTransparency(a);
  }
  void Idle() {
    assert(!!m_viewer);
    m_viewer->Idle();
  }
  PyGraphHandle DrawText(std::string text, float x, float y, float fontsize, py::object pycolor) {
    OpenRAVE::Vector color = OpenRAVE::Vector(py::extract<float>(pycolor[0]), py::extract<float>(pycolor[1]), py::extract<float>(pycolor[2]), py::extract<float>(pycolor[3]));
    return PyGraphHandle(m_viewer->drawtext(text, x, y, fontsize, color));
  }
  void AddManipulatorControl(py::object py_manip) {
    RobotBase::ManipulatorPtr manip = GetCppManip(py_manip, m_viewer->GetEnv());
    SetUserData(*manip->GetRobot(), "ManipulatorControl", UserDataPtr(new ManipulatorControl(manip, m_viewer)));
  }
  void AddDriveControl(py::object py_robot) {
    RobotBasePtr robot = GetCppRobot(py_robot, m_viewer->GetEnv());
    SetUserData(*robot, "DriveControl", UserDataPtr(new DriveControl(robot, m_viewer)));
  }
  //void AddInteractiveMarker(py::object pose) {
  //  OpenRAVE::Transform T;
  //  return UserDataPtr(new InteractiveMarker(T, m_viewer));
  //}
  void AddKeyCallback(py::object key, py::object py_fn) {
    OSGViewer::KeyCallback f = boost::bind(CallPyFunc, py_fn);
    m_viewer->AddKeyCallback(py::extract<int>(key), f);
  }
  
private:
  OSGViewerPtr m_viewer;
  PyOSGViewer() {}
};
PyOSGViewer PyGetViewer(py::object py_env) {
  EnvironmentBasePtr env = GetCppEnv(py_env);
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);
  ALWAYS_ASSERT(!!viewer);
  return PyOSGViewer(viewer);
}

void translate_runtime_error(std::runtime_error const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BodyVsAllDefaults, PyCollisionChecker::BodyVsAll, 1, 2);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(BodyVsBodyDefaults, PyCollisionChecker::BodyVsBody, 2, 3);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(CastVsAllDefaults, PyCollisionChecker::CastVsAll, 3, 5);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(MultiCastVsAllDefaults, PyCollisionChecker::MultiCastVsAll, 2, 5);

BOOST_PYTHON_MODULE(ctrajoptpy) {

  np_mod = py::import("numpy");

  py::object openravepy = py::import("openravepy");

  string pyversion = py::extract<string>(openravepy.attr("__version__"));
  if (OPENRAVE_VERSION_STRING != pyversion) {
    PRINT_AND_THROW("the openrave on your pythonpath is different from the openrave version that trajopt links to!");
  }

  py::register_exception_translator<std::runtime_error>(&translate_runtime_error);

  py::class_<PyTrajOptProb>("TrajOptProb", py::no_init)
      .def("GetDOFIndices", &PyTrajOptProb::GetDOFIndices)
      .def("SetRobotActiveDOFs", &PyTrajOptProb::SetRobotActiveDOFs, "Sets the active DOFs of the robot to the DOFs in the optimization problem")
      .def("AddConstraint", &PyTrajOptProb::AddConstraint1, "Add constraint from python function (using numerical differentiation)", (py::arg("f"),"var_ijs","constraint_type","name"))
      .def("AddConstraint", &PyTrajOptProb::AddConstraint2, "Add constraint from python error function and analytic derivative", (py::arg("f"),"dfdx","var_ijs","constraint_type","name"))
      .def("AddCost", &PyTrajOptProb::AddCost1, "Add cost from python scalar-valued function (using numerical differentiation)", (py::arg("func"),"var_ijs", "name"))
      .def("AddErrorCost", &PyTrajOptProb::AddErrCost1, "Add error cost from python vector-valued error function (using numerical differentiation)", (py::arg("f"),"var_ijs","penalty_type","name"))
      .def("AddErrorCost", &PyTrajOptProb::AddErrCost2, "Add error cost from python vector-valued error function and analytic derivative",(py::arg("f"),"dfdx","var_ijs","penalty_type","name"))
  ;
  py::def("SetInteractive", &SetInteractive, "if True, pause and plot every iteration");
  py::def("ConstructProblem", &PyConstructProblem, "create problem from JSON string");
  py::def("OptimizeProblem", &PyOptimizeProblem);

  py::class_<PyTrajOptResult>("TrajOptResult", py::no_init)
      .def("GetCosts", &PyTrajOptResult::GetCosts)
      .def("GetConstraints", &PyTrajOptResult::GetConstraints)
      .def("GetTraj", &PyTrajOptResult::GetTraj)
      .def("GetCollisions", &PyTrajOptResult::GetCollisions)
      .def("__str__", &PyTrajOptResult::__str__)
      ;

  py::class_<PyCollisionChecker>("CollisionChecker", py::no_init)
      .def("AllVsAll", &PyCollisionChecker::AllVsAll)
      .def("BodyVsAll", &PyCollisionChecker::BodyVsAll, BodyVsAllDefaults())
      .def("BodyVsBody", &PyCollisionChecker::BodyVsBody, BodyVsBodyDefaults())
      .def("BodiesVsBodies", &PyCollisionChecker::BodiesVsBodies)
      .def("CastVsAll", &PyCollisionChecker::CastVsAll, CastVsAllDefaults())
      .def("MultiCastVsAll", &PyCollisionChecker::MultiCastVsAll, MultiCastVsAllDefaults())
      .def("PlotCollisionGeometry", &PyCollisionChecker::PlotCollisionGeometry)
      .def("ExcludeCollisionPair", &PyCollisionChecker::ExcludeCollisionPair)
      .def("IncludeCollisionPair", &PyCollisionChecker::IncludeCollisionPair)
      .def("SetContactDistance", &PyCollisionChecker::SetContactDistance)
      .def("GetContactDistance", &PyCollisionChecker::GetContactDistance)
      ;
  py::def("GetCollisionChecker", &PyGetCollisionChecker);
  py::class_<PyCollision>("Collision", py::no_init)
     .def("GetDistance", &PyCollision::GetDistance)
     .def("GetNormal", &PyCollision::GetNormal)
     .def("GetPtA", &PyCollision::GetPtA)
     .def("GetPtB", &PyCollision::GetPtB)
     .def("GetLinkAName", &PyCollision::GetLinkAName)
     .def("GetLinkBName", &PyCollision::GetLinkBName)
     .def("GetLinkAParentName", &PyCollision::GetLinkAParentName)
     .def("GetLinkBParentName", &PyCollision::GetLinkBParentName)
     .def("GetMultiCastAlphas", &PyCollision::GetMultiCastAlphas)
     .def("GetMultiCastIndices", &PyCollision::GetMultiCastIndices)
    ;
  py::class_< PyGraphHandle >("GraphHandle", py::no_init)
     .def("SetTransparency", &PyGraphHandle::SetTransparency1)
     ;

  py::class_< PyOSGViewer >("OSGViewer", py::no_init)
     .def("UpdateSceneData", &PyOSGViewer::UpdateSceneData)
     .def("Step", &PyOSGViewer::Step)
     .def("PlotKinBody", &PyOSGViewer::PlotKinBody)
     .def("PlotLink", &PyOSGViewer::PlotLink)
     .def("SetTransparency", &PyOSGViewer::SetTransparency)
     .def("SetAllTransparency", &PyOSGViewer::SetAllTransparency)
     .def("Idle", &PyOSGViewer::Idle)
     .def("DrawText", &PyOSGViewer::DrawText)
     .def("AddManipulatorControl",&PyOSGViewer::AddManipulatorControl)
     .def("AddDriveControl",&PyOSGViewer::AddDriveControl)
     .def("AddKeyCallback", &PyOSGViewer::AddKeyCallback)
    ;
  py::def("GetViewer", &PyGetViewer, "Get OSG viewer for environment or create a new one");

}
