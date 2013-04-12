#include <openrave-core.h>
#include <openrave/openrave.h>

#include "trajopt/problem_description.hpp"
#include "trajopt/kinematic_constraints.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "utils/eigen_conversions.hpp"
#include "sco/expr_op_overloads.hpp"
#include "sco/optimizers.hpp"
#include "quat_ops.hpp"
#include <boost/assign.hpp>
#include "dynamics_utils.hpp"
#include "sco/modeling_utils.hpp"
#include "osgviewer/osgviewer.hpp"
#include "trajopt/collision_avoidance.hpp"
using namespace boost::assign;
using namespace OpenRAVE;
using namespace trajopt;
using namespace sco;
using namespace Eigen;
using namespace std;

template<typename T>
void extend(vector<T>& a, const vector<T>& b) {
  a.insert(a.end(), b.begin(), b.end());
}

VectorXd concat(const VectorXd& a, const VectorXd& b) {
  VectorXd out(a.size()+b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}


struct IncrementalRB : public Configuration {
  KinBodyPtr m_body;
  OpenRAVE::Vector m_q;
  OpenRAVE::Vector m_r;


  IncrementalRB(KinBodyPtr body) : m_body(body), m_r(0,0,0) {}
  virtual void SetDOFValues(const DblVec& dofs) {
    OR::Transform T;
    m_r = T.trans = OR::Vector(dofs[0], dofs[1], dofs[2]);
    T.rot = geometry::quatMultiply(geometry::quatFromAxisAngle(OpenRAVE::Vector(dofs[3],dofs[4], dofs[5])), m_q);
    m_body->SetTransform(T);
  }
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const {
    lower = DblVec(6, -INFINITY);
    upper = DblVec(6, INFINITY);
  };
  virtual DblVec GetDOFValues() {
    DblVec out(6);
    OpenRAVE::Transform T = m_body->GetTransform();
    out[0] = T.trans.x;
    out[1] = T.trans.y;
    out[2] = T.trans.z;
    out[3] = m_r[0];
    out[4] = m_r[1];
    out[5] = m_r[2];
    return out;
  };
  virtual int GetDOF() const {
    return 6;
  }
  virtual OpenRAVE::EnvironmentBasePtr GetEnv() {
    return m_body->GetEnv();
  }
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const {
    MatrixXd out(3,6);
    out.leftCols(3) = Matrix3d::Identity();
    assert(link_ind == 0);
    KinBody::LinkPtr link = m_body->GetLinks()[link_ind];
    OpenRAVE::Vector dr = pt - link->GetTransform().trans;
    double matdata[9] = {0, dr[2], -dr[1],      -dr[2], 0, dr[0],      dr[1], -dr[0],0};
    out.rightCols(3) = Eigen::Map<MatrixXd>(matdata, 3, 3);
    return out;
  }
  virtual DblMatrix RotationJacobian(int link_ind) const {
    PRINT_AND_THROW("not implemented");
  }
  virtual bool DoesAffect(const KinBody::Link& link) {
    BOOST_FOREACH(const KinBody::LinkPtr& link1, m_body->GetLinks()) {if (link1.get() == &link) return true;}
    return false;
  }
  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() {
    return m_body->GetLinks();
  }
  virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) {
    links = GetAffectedLinks();
    link_inds.resize(links.size());
    for (int i=0; i < links.size(); ++i) link_inds.push_back(links[i]->GetIndex());
  }
  virtual DblVec RandomDOFValues() {
    return toDblVec(VectorXd::Random(6));
  }
};
typedef boost::shared_ptr<IncrementalRB> IncrementalRBPtr;

class AngVelCost : public Cost {
public:
  AngVelCost(const MatrixXd& q, const VarArray& r, double dt) : q_(q), r_(r), dt_(dt) {}
  double value(const DblVec& x) {
    MatrixXd rvals = getTraj(x, r_);
    MatrixXd qnew(q_.rows(), q_.cols());
    for (int i=0; i < qnew.rows(); ++i) {
      qnew.row(i) = quatMult(quatExp(rvals.row(i)), q_.row(i));
    }
    MatrixXd wvals = getW(qnew, dt_);
    cout << wvals << endl;
    return wvals.array().square().sum();
  }
  ConvexObjectivePtr convex(const DblVec& x, Model* model) {
    ConvexObjectivePtr out(new ConvexObjective(model));
    MatrixXd wvals = getW(q_, dt_);
    for (int i=0; i < wvals.rows(); ++i) {
      for (int j=0; j < wvals.cols(); ++j) {
        out->addQuadExpr(exprSquare( (r_(i+1,j) - r_(i,j))*(1/dt_)  + wvals(i,j) ));
      }
    }
    return out;
  }
  const MatrixXd& q_;
  VarArray r_;
  double dt_;
};


struct PositionRB : public Configuration {
  KinBodyPtr m_body;
  KinBody::LinkPtr m_link;
  
  PositionRB(KinBodyPtr body, KinBody::LinkPtr link) : m_body(body), m_link(link) {}
  virtual void SetDOFValues(const DblVec& dofs) {
    OpenRAVE::Transform T = m_link->GetTransform();
    T.trans.x = dofs[0];
    T.trans.y = dofs[1];
    T.trans.z = dofs[2];
    m_body->SetTransform(T);    
  }
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const {
    lower = DblVec(3, -INFINITY);
    upper = DblVec(3, INFINITY);
  };
  virtual DblVec GetDOFValues() {
    DblVec out(3);
    OpenRAVE::Transform T = m_link->GetTransform();
    out[0] = T.trans.x;
    out[1] = T.trans.y;
    out[2] = T.trans.z;
    return out;
  };
  virtual int GetDOF() const {
    return 3;
  }
  virtual OpenRAVE::EnvironmentBasePtr GetEnv() {
    return m_link->GetParent()->GetEnv();
  }
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const {
    return Matrix3d::Identity();
  }
  virtual DblMatrix RotationJacobian(int link_ind) const {
    PRINT_AND_THROW("not implemented");
  }
  virtual bool DoesAffect(const KinBody::Link& link) {
    return &link == m_link.get();
  }
  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() {
    return vector<KinBody::LinkPtr>(1, m_link);
  }
  virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) {
    links = GetAffectedLinks();
    link_inds.resize(1);
    link_inds[0] = 0;
  }
  virtual DblVec RandomDOFValues() {
    return toDblVec(VectorXd::Random(3));
  }
};


struct PushingProblem : public OptProb {
public:
  
  int m_nSteps;
    
  ConfigurationPtr m_robotConfig;
  vector<IncrementalRBPtr> m_objConfigs;
  KinBody::LinkPtr m_robotLink, m_objLink;
  
  MatrixX2d m_facePoly; // polygon sides
  OpenRAVE::Transform m_T_lf;
  
  VarArray m_th; // robot
  VarArray m_p; // position
  MatrixXd m_q; // quaternion
  VarArray m_r; // incremental rotation

  VarArray m_cp; // local contact points inside face
  VarArray m_f; // normal force
    
  void Callback(const DblVec&);
  void AddFaceContactConstraints();
  void AddQuasistaticsConstraints();
  PushingProblem(int nSteps, ConfigurationPtr robotConfig, KinBodyPtr objBody,
    KinBody::LinkPtr robotLink, const MatrixX2d& facePoly, const OpenRAVE::Transform& T_lf);
};

OpenRAVE::Vector toRaveQuat(const Vector4d& v) {
  return OpenRAVE::Vector(v[0], v[1], v[2], v[3]);
}
void PushingProblem::Callback(const DblVec& x) {


  MatrixXd rvals = getTraj(x, m_r);
  // update quaternions
  for (int i=0; i < m_q.rows(); ++i) {
    m_q.row(i) = quatMult(m_q.row(i), quatExp(rvals.row(i)));
    m_objConfigs[i]->m_q = toRaveQuat(m_q.row(i));
  }




  
  RobotBasePtr robot = boost::dynamic_pointer_cast<RobotAndDOF>(m_robotConfig)->GetRobot();
  EnvironmentBasePtr env = robot->GetEnv();
  KinBodyPtr body = m_objLink->GetParent();
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);

  assert(robot);
  assert(body);

  // cout << "object transform: " << body->GetTransform() << endl;
  

  vector<GraphHandlePtr> handles;
  BOOST_FOREACH(const CostPtr& cost, getCosts()) {
    if (Plotter* plotter = dynamic_cast<Plotter*>(cost.get())) {
      plotter->Plot(x, *env, handles);
      plotter->Plot(x, *env, handles);
    }
  }
  BOOST_FOREACH(const ConstraintPtr& cnt, getConstraints()) {
    if (Plotter* plotter = dynamic_cast<Plotter*>(cnt.get())) {
      plotter->Plot(x, *env, handles);
    }
  }
  TrajArray robotTraj = getTraj(x, m_th);
  TrajArray objTraj = getTraj(x, m_p);
  TrajArray localCPTraj = getTraj(x, m_cp);
  
  // cout << "robot traj:" << endl << robotTraj << endl;
  // cout << "obj traj:" << endl << objTraj << endl;
  // cout << "cp traj:" << endl << localCPTraj << endl;


  for (int i=0; i < m_nSteps; ++i) {
    m_robotConfig->SetDOFValues(toDblVec(robotTraj.row(i)));
    m_objConfigs[i]->SetDOFValues(toDblVec(objTraj.row(i)));
    handles.push_back(viewer->PlotKinBody(robot));
    SetTransparency(handles.back(), .35);    
    handles.push_back(viewer->PlotKinBody(body));
    SetTransparency(handles.back(), .35);
    // todo plot contacts
  }
  viewer->Idle();

  
}
  



struct QuasistaticsErrorCalculator : public VectorOfVector {


  ConfigurationPtr m_config;
  KinBody::LinkPtr m_link;
  int m_nDof;
  OpenRAVE::Vector m_normalLocal; // points OUT OF object
  OpenRAVE::Transform m_T_lf;
  VectorXd m_mass;
  
  QuasistaticsErrorCalculator(ConfigurationPtr config, KinBody::LinkPtr link, const OpenRAVE::Transform& T_lf) 
    : m_config(config), m_link(link), m_nDof(config->GetDOF()), m_normalLocal(0,0,1), m_T_lf(T_lf),
      m_mass(VectorXd::Ones(config->GetDOF())) // xxx
   {}

  /**
  input vector: dofs0 dofs1 ptlocalxy f1
  the face frame has the z axis pointing out of the face, and is centered on the face.
  ptlocalxy is in the frame of the face (z coord is zero)  
  */
  
  VectorXd operator()(const VectorXd& x) const {
    
    Configuration::SaverPtr saver = m_config->Save();
    
    VectorXd dofs0 = x.topRows(m_nDof);
    VectorXd dofs1 = x.middleRows(m_nDof, m_nDof);
    OpenRAVE::Vector ptLocal;
    ptLocal.x = x(2*m_nDof);
    ptLocal.y = x(2*m_nDof+1);
    double f = x(x.size()-1);
    
    m_config->SetDOFValues(toDblVec(dofs1)); // implicit euler, so we assume force is applied at end of timestep
    OpenRAVE::Transform T_wl = m_link->GetTransform();
    Vector3d forceDirectionWorld = - toVector3d(geometry::quatRotate(geometry::quatMultiply(T_wl.rot, m_T_lf.rot), m_normalLocal));    
    OpenRAVE::Vector ptWorld = T_wl * m_T_lf * ptLocal;
        
    VectorXd out(m_nDof);

    VectorXd wrench(6);
    Vector3d forceVector = forceDirectionWorld * f;
    wrench.topRows(3) = forceVector;
    wrench.bottomRows(3)  = (toVector3d(ptWorld) - toVector3d(T_wl.trans)).cross(forceVector);
    
    
    for (int i=0; i < out.size(); ++i) {
      out(i) = m_mass(i) * (dofs1(i) - dofs0(i)) - wrench(i);
    }
    
    return out;
  }
};

struct Quasistatics : public ConstraintFromFunc {
  Quasistatics(ConfigurationPtr config, KinBody::LinkPtr link, OpenRAVE::Transform T_lf, const VarVector& vars) 
  : ConstraintFromFunc(VectorOfVectorPtr(new QuasistaticsErrorCalculator(config, link, T_lf)), vars, EQ, "quasistatics") {}
};

struct FaceContactErrorCalculator : public VectorOfVector {
  KinBody::LinkPtr m_linkA, m_linkB;
  OpenRAVE::Vector m_ptLocalA;
  OpenRAVE::Transform m_T_lf_B;
  ConfigurationPtr m_config;
  int m_nDof;
  FaceContactErrorCalculator(ConfigurationPtr config, KinBody::LinkPtr linkA, KinBody::LinkPtr linkB, OpenRAVE::Vector ptA, OpenRAVE::Transform T_lf_B) 
    : m_config(config), m_linkA(linkA), m_linkB(linkB), m_ptLocalA(ptA), m_T_lf_B(T_lf_B), m_nDof(config->GetDOF()) {}
  
  // input: dof values (for composite config, presumably), local points in face
  
  VectorXd operator()(const VectorXd& x) const {
    
    DblVec dofs(x.data(), x.data()+m_nDof);
    OpenRAVE::Vector ptLocalB;
    ptLocalB.x = x(m_nDof);
    ptLocalB.y = x(m_nDof+1);
    
    m_config->SetDOFValues(dofs);
    OpenRAVE::Vector ptWorldA = m_linkA->GetTransform() * m_ptLocalA;
    OpenRAVE::Vector ptWorldB = m_linkB->GetTransform() * m_T_lf_B * ptLocalB;
    return toVector3d(ptWorldA - ptWorldB);
  }
  
  
};

struct FaceContact : public ConstraintFromFunc, public Plotter {
  FaceContact(ConfigurationPtr config, const VarVector& dofvars, const VarVector& ptvars, 
    KinBody::LinkPtr linkA, KinBody::LinkPtr linkB, const OpenRAVE::Vector& ptA, const OpenRAVE::Transform& T_lf_B) 
  : ConstraintFromFunc(VectorOfVectorPtr(new FaceContactErrorCalculator(config, linkA, linkB, ptA, T_lf_B)), concat(dofvars, ptvars), EQ, "facecontact") {}    

  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
  
};

void FaceContact::Plot(const DblVec& x0, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  
  VectorXd x = getVec(x0, vars_);
  FaceContactErrorCalculator* calc = static_cast<FaceContactErrorCalculator*>(f_.get());

  DblVec dofs(x.data(), x.data()+calc->m_nDof);
  
  // cout << "T_lf " << calc->m_T_lf_B << endl;
  // cout << "T_wb " << calc->m_linkB->GetTransform() << endl;
  OpenRAVE::Vector ptLocalB;
  ptLocalB.x = x(calc->m_nDof);
  ptLocalB.y = x(calc->m_nDof+1);

  calc->m_config->SetDOFValues(dofs);
  OpenRAVE::Vector ptWorldA = calc->m_linkA->GetTransform() * calc->m_ptLocalA;
  OpenRAVE::Vector ptWorldB = calc->m_linkB->GetTransform() * calc->m_T_lf_B * ptLocalB;
  // cout << "A: " << ptWorldA << " B: " << ptWorldB << endl;
  handles.push_back(env.drawarrow(ptWorldA, ptWorldB, .01, OR::Vector(1,0,1,1)));
  OpenRAVE::Transform T;
  T.trans = ptWorldA;
  handles.push_back(OSGViewer::GetOrCreate(env.shared_from_this())->PlotAxes(T, .05));  
  T.trans = ptWorldB;
  handles.push_back(OSGViewer::GetOrCreate(env.shared_from_this())->PlotAxes(T, .05));
}




void Callback(OptProb* prob, DblVec& x) {
  dynamic_cast<PushingProblem*>(prob)->Callback(x);
}

extern void PolygonToEquations(const MatrixX2d& pts, MatrixX2d& ab, VectorXd& c);

PushingProblem::PushingProblem(int nSteps, ConfigurationPtr robotConfig, KinBodyPtr objBody,
  KinBody::LinkPtr robotLink, const MatrixX2d& facePoly, const OpenRAVE::Transform& T_lf)
: m_nSteps(nSteps),
  m_robotConfig(robotConfig),
  m_robotLink(robotLink),
  m_objLink(objBody->GetLinks()[0]),
  m_facePoly(facePoly),
  m_T_lf(T_lf)
{


 for (int i=0; i < nSteps; ++i) {
   m_objConfigs.push_back(IncrementalRBPtr(new IncrementalRB(objBody)));
 }

  vector<VarArray*> arrs; arrs += &m_th, &m_p, &m_f, &m_cp;
  vector<string> prefixes; prefixes += "th","p","f","cp";
  vector<int> cols; cols += robotConfig->GetDOF(),m_objConfigs[0]->GetDOF(),1, 2;
  AddVarArrays(*this, m_nSteps, cols, prefixes, arrs);


  m_q.resize(nSteps,4);
  for (int i=0; i < nSteps; ++i) {
    m_q.row(i) = Vector4d(1,0,0,0).transpose();
  }


  
  m_r = m_p.block(0,3,m_p.rows(), 3);

  VectorXd c;
  MatrixX2d ab;
  PolygonToEquations(facePoly, ab, c)  ;
  // cout << facePoly << endl;
  // cout << "ab:" << endl << ab << endl;
  // cout << "c:"<<c.transpose() << endl;
  
   // todo: local contact point should be fixed!
  
  for (int i=0; i < m_cp.rows(); ++i) {
    for (int j=0; j < ab.rows(); ++j) {
      addLinearConstr(ab(j,0) * m_cp(i,0) + ab(j,1) * m_cp(i,1) + c(j), INEQ);      
    }
  }

  addCost(CostPtr(new JointVelCost(m_th, VectorXd::Ones(m_th.cols()))));
  addCost(CostPtr(new AngVelCost(m_q, m_r,1)));
  AddFaceContactConstraints();
  AddQuasistaticsConstraints();
}
void PushingProblem::AddFaceContactConstraints() {
  // XXX
  OpenRAVE::Vector ptA(0,0,0);
  for (int i=0; i < m_nSteps; ++i) {    
    ConfigurationPtr robotAndObjConfig(new CompositeConfig(m_robotConfig, m_objConfigs[i]));
//    addCost(CostPtr(new CollisionCost(.01, 10, robotAndObjConfig, concat(m_th.row(i), m_p.row(i)))));
    addConstr(ConstraintPtr(new FaceContact(robotAndObjConfig, concat(m_th.row(i), m_p.row(i)), m_cp.row(i), m_robotLink, m_objLink, ptA, m_T_lf)));
  }
}
void PushingProblem::AddQuasistaticsConstraints() {
  for (int i=0; i < m_nSteps-1; ++i) {
    VarVector vars;
    extend(vars, m_p.row(i));
    extend(vars, m_p.row(i+1));
    extend(vars, m_cp.row(i));
    extend(vars, m_f.row(i));
    addConstr(ConstraintPtr(new Quasistatics(m_objConfigs[i], m_objLink, m_T_lf, vars)));
  }
}

/**
stages of getting stuff to work:
1. just set up problem.
2. no collisions problem
3. face contact constraint
4. face contact + dynamics

gotta figure out where contat point is on robot

to plot:
- plot contact point location
- plot force strength

*/

void setVec(DblVec& x, const VarVector& vars, const VectorXd& vals) {
  assert(vars.size() == vals.size());
  for (int i=0; i < vars.size(); ++i) {
    x[vars[i].var_rep->index] = vals[i];
  }
}


int main(int argc, char** argv) {
  int nSteps = 10;
  
  RaveInitialize(false, Level_Debug);
  OpenRAVE::EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  env->Load(string(DATA_DIR)+"/three_links_obstacles.env.xml");
  OSGViewerPtr viewer(new OSGViewer(env));
  viewer->UpdateSceneData();
  env->AddViewer(viewer);
 
 
  RobotBasePtr robot = GetRobotByName(*env, "3DOFRobot");
  assert(robot);
  KinBodyPtr obj = GetBodyByName(*env, "obstacle");
  assert(obj);
  
  KinBody::LinkPtr objLink = obj->GetLink("obstacle");
  KinBody::LinkPtr robotLink = robot->GetLink("Finger");
  assert(objLink);
  assert(robotLink);
 
  vector<int> robotDofInds; // xxx todo fill in
  robotDofInds += 0,1,2;
  RobotAndDOFPtr robotConfig(new RobotAndDOF(robot, robotDofInds));
  
  MatrixX2d facePoly(4,2);
  facePoly << -.1,-.1,  -.1,.1,  .1,.1,  .1,-.1;
  
  OpenRAVE::Transform T_lf; 
  T_lf.trans.x = .1;
  T_lf.rot = OpenRAVE::geometry::quatFromAxisAngle(OpenRAVE::Vector(0,1,0), -M_PI/2);
  
  vector<GraphHandlePtr> handles;  
  handles.push_back(viewer->PlotAxes(obj->GetLinks()[0]->GetTransform() * T_lf, .1));
  viewer->Idle();  
  
  boost::shared_ptr<PushingProblem> prob(new PushingProblem(nSteps, robotConfig, obj, robotLink, facePoly, T_lf));

  DblVec xinit(prob->getNumVars(), 0);

  VectorXd startPt(6); startPt << 0.05, 0.25, 0,   0,0,0;
  VectorXd endPt(6); endPt <<  -0.05, 0.25,0,      0,0,0;
  for (int j=0; j < 6; ++j) {
    setVec(xinit, prob->m_p.col(j), VectorXd::LinSpaced(nSteps, startPt(j), endPt(j)));
    prob->addLinearConstr(exprSub(AffExpr(prob->m_p(0,j)), startPt[j]), EQ);    
    prob->addLinearConstr(exprSub(AffExpr(prob->m_p(nSteps-1,j)), endPt[j]), EQ);    
  }

  
  BasicTrustRegionSQP opt(prob);
  opt.initialize(xinit);
  opt.addCallback(&Callback);
  OptStatus status = opt.optimize();
  RaveDestroy();
}
