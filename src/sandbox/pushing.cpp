#include <openrave-core.h>
#include <openrave/openrave.h>
#include <boost/assign.hpp>

#include "trajopt/problem_description.hpp"
#include "trajopt/kinematic_terms.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "trajopt/trajectory_costs.hpp"
#include "utils/eigen_conversions.hpp"
#include "sco/expr_op_overloads.hpp"
#include "sco/optimizers.hpp"
#include "quat_ops.hpp"
#include "dynamics_utils.hpp"
#include "sco/modeling_utils.hpp"
#include "osgviewer/osgviewer.hpp"
#include "trajopt/collision_terms.hpp"
#include "utils/stl_to_string.hpp"
#include "utils/config.hpp"
#include "o3.hpp"
#include "static_object.hpp"
#include "incremental_rb.hpp"
#include "utils/logging.hpp"
#include "composite_config.hpp"
#include "sco/expr_ops.hpp"
using namespace boost::assign;
using namespace OpenRAVE;
using namespace trajopt;
using namespace sco;
using namespace Eigen;
using namespace std;
using namespace util;

extern void PolygonToEquations(const MatrixX2d& pts, MatrixX2d& ab,VectorXd& c);


double mu = 1; // TODO make it a parameter
bool idle = false;
bool constrain_all_poses = true;
int nSteps=10;



enum ContactType {
  SLIDING, STICKING
};

struct Face {
  MatrixX2d poly;
  OpenRAVE::Transform Tlf;
  KinBody::LinkPtr link;
  Face() {}
  Face(const MatrixX2d& _poly, const OpenRAVE::Transform& _Tlf, KinBody::LinkPtr _link) : poly(_poly), Tlf(_Tlf), link(_link) {}
};


struct Contact {
  ContactType m_type;
  // contact at a single timestep
  VarVector m_dofvars;
  VarVector m_ptA, m_ptB; // local contacts inside faces
  Var m_fn; // normal force B to A
  VarVector m_fr; // friction force B to A
  ConfigurationPtr m_config; //composite config
  Face m_faceA, m_faceB;
  Contact(ContactType type, ConfigurationPtr config, const Face& faceA, const Face& faceB, const VarVector& dofvars, 
      const VarVector& ptA, const VarVector& ptB, const Var& fn, const VarVector& fr)
  : m_type(type), m_dofvars(dofvars),m_ptA(ptA), m_ptB(ptB), m_fn(fn), m_fr(fr), m_config(config),  m_faceA(faceA), m_faceB(faceB) {}
  virtual ~Contact() {}
};
typedef boost::shared_ptr<Contact> ContactPtr;


struct FaceContactErrorCalculator: public VectorOfVector {
  ConfigurationPtr m_config;
  Face m_faceA, m_faceB;
  int m_nDof;
  FaceContactErrorCalculator(ConfigurationPtr config, const Face& faceA, const Face& faceB) :
    m_config(config), m_faceA(faceA), m_faceB(faceB), m_nDof(m_config->GetDOF()) {}


  void CalcWorldPoints(const VectorXd& x, OR::Vector& ptWorldA, OR::Vector& ptWorldB) const {
    OR::Vector nA, nB;
    CalcWorldPointsAndNormal(x, ptWorldA, ptWorldB, nA, nB);
  }
  void CalcWorldPointsAndNormal(const VectorXd& x, OR::Vector& ptWorldA, OR::Vector& ptWorldB, OR::Vector& nWorldA, OR::Vector& nWorldB)  const {
    DblVec dofs(x.data(), x.data() + m_nDof);
    OpenRAVE::Vector ptLocalA, ptLocalB;
    ptLocalA.x = x(m_nDof);
    ptLocalA.y = x(m_nDof + 1);
    ptLocalB.x = x(m_nDof + 2);
    ptLocalB.y = x(m_nDof + 3);

    m_config->SetDOFValues(dofs);
    ptWorldA = m_faceA.link->GetTransform() * (m_faceA.Tlf * ptLocalA);
    ptWorldB = m_faceB.link->GetTransform() * (m_faceB.Tlf * ptLocalB);
    
    nWorldA = geometry::quatRotate(geometry::quatMultiply(m_faceA.link->GetTransform().rot,m_faceA.Tlf.rot), OR::Vector(0,0,1));
    nWorldB = geometry::quatRotate(geometry::quatMultiply(m_faceB.link->GetTransform().rot,m_faceB.Tlf.rot), OR::Vector(0,0,1));
    
  }
  VectorXd operator()(const VectorXd& x) const {
    m_config->SetDOFValues(toDblVec(x.topRows(m_nDof)));
    OR::Vector ptWorldA, ptWorldB, nWorldA, nWorldB;
    CalcWorldPoints(x, ptWorldA, ptWorldB);
     return toVector3d(ptWorldA - ptWorldB);

  }

};

struct FaceContactConstraint: public ConstraintFromFunc, public Plotter {
  FaceContactConstraint(ConfigurationPtr config, const VarVector& dofvars, const VarVector& ptAvars, const VarVector& ptBvars,
      const Face& faceA, const Face& faceB) :
      ConstraintFromFunc(VectorOfVectorPtr(new FaceContactErrorCalculator(config, faceA, faceB)),
          concat(dofvars, ptAvars, ptBvars), VectorXd::Zero(0), EQ, faceA.link->GetName()+"_"+faceB.link->GetName()+"contact") {}

  void Plot(const DblVec& xvec, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
};


void FaceContactConstraint::Plot(const DblVec& xvec, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  FaceContactErrorCalculator* calc = static_cast<FaceContactErrorCalculator*>(f_.get());
  OR::Vector ptA, ptB;
  VectorXd x = getVec(xvec, vars_);
  calc->m_config->SetDOFValues(toDblVec(x.topRows(calc->m_nDof)));
  calc->CalcWorldPoints(x, ptA, ptB);
  handles.push_back(env.drawarrow(ptA, ptB, .001, OR::Vector(1, 0, 1, 1)));
}




struct ForceBalanceConstraint : public EqConstraint {

  ConfigurationPtr m_config;
  KinBody::LinkPtr m_link;
  vector<ContactPtr> m_contacts;
  VarVector m_dofs;

  ForceBalanceConstraint(ConfigurationPtr config, KinBody::LinkPtr link, const vector<ContactPtr>& contacts, const VarVector& dofs) : m_config(config), m_link(link), m_contacts(contacts), m_dofs(dofs) {
    name_ = "forcebalance";
  }

  AffExprVector CalcWrenchExpr(const DblVec& x, Contact& c);

  DblVec CalcWrenchValue(const DblVec& x, Contact& c);

  vector<double> value(const vector<double>& x);

  ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
};


AffExprVector ForceBalanceConstraint::CalcWrenchExpr(const DblVec& x, Contact& c) {
  DblVec dofs =getDblVec(x, c.m_dofvars);
  c.m_config->SetDOFValues(dofs);

  OpenRAVE::Transform Tlf = c.m_faceA.Tlf;
  AffExprVector fExprFace(3), rExprFace(3);
  if (c.m_faceA.link == m_link) {
    fExprFace[0] = -AffExpr(c.m_fr[0]);
    fExprFace[1] = -AffExpr(c.m_fr[1]);
    fExprFace[2] = -AffExpr(c.m_fn);
    rExprFace[0] = AffExpr(c.m_ptA[0]);
    rExprFace[1] = AffExpr(c.m_ptA[1]);
  }
  else if (c.m_faceB.link == m_link) {
    fExprFace[0] =  AffExpr(c.m_fr[0]);
    fExprFace[1] =  AffExpr(c.m_fr[1]);
    fExprFace[2] =  AffExpr(c.m_fn);
    rExprFace[0] = AffExpr(c.m_ptB[0]);
    rExprFace[1] = AffExpr(c.m_ptB[1]);
  }
  else assert(0);


  AffExprVector fExprLink = rotateExpr(Tlf.rot, fExprFace);
  AffExprVector rExprLink = transformExpr(Tlf, rExprFace);  

  AffExprVector tExprLink = linearizedCrossProduct(x, rExprLink, fExprLink);
  // cout << c.m_faceA.link->GetName() << c.m_faceB.link->GetName() << endl;
  // cout << "rexprlink" << rExprLink[0].value(x) << " " << rExprLink[1].value(x)<< " "<<rExprLink[2].value(x)<< endl;
  // cout << "rexprface" << rExprFace[0].value(x) << " " << rExprFace[1].value(x)<< " "<<rExprFace[2].value(x)<< endl;
  // cout << "fexprlink" << fExprLink[0].value(x) << " " << fExprLink[1].value(x)<< " "<<fExprLink[2].value(x)<< endl;
  // cout << "fexprface" << fExprFace[0].value(x) << " " << fExprFace[1].value(x)<< " "<<fExprFace[2].value(x)<< endl;
  // cout << "Tlf" << c.m_faceA.Tlf << endl;

  return concat(fExprLink, tExprLink);
}

DblVec ForceBalanceConstraint::CalcWrenchValue(const DblVec& x, Contact& c) {
  DblVec dofs =getDblVec(x, c.m_dofvars);
  
  c.m_config->SetDOFValues(dofs);

  OpenRAVE::Transform Tlf = c.m_faceA.Tlf;
  OR::Vector fFace, rFace;
  if (c.m_faceA.link == m_link) {
    fFace[0] = -c.m_fr[0].value(x);
    fFace[1] = -c.m_fr[1].value(x);
    fFace[2] = -c.m_fn.value(x);
    rFace[0] = c.m_ptA[0].value(x);
    rFace[1] = c.m_ptA[1].value(x);
  }
  else if (c.m_faceB.link == m_link) {
    fFace[0] =  c.m_fr[0].value(x);
    fFace[1] =  c.m_fr[1].value(x);
    fFace[2] = c.m_fn.value(x);
    rFace[0] = c.m_ptB[0].value(x);
    rFace[1] = c.m_ptB[1].value(x);
  }
  else assert(0);

  OR::Vector fLink = OR::geometry::quatRotate(Tlf.rot, fFace);
  OR::Vector rLink = Tlf * rFace;

  OR::Vector tLink = rLink.cross(fLink);

  DblVec out(6);
  out[0] = fLink[0];
  out[1] = fLink[1];
  out[2] = fLink[2];
  out[3] = tLink[0];
  out[4] = tLink[1];
  out[5] = tLink[2];
  return out;
}

vector<double> ForceBalanceConstraint::value(const vector<double>& x) {
  DblVec ftotal(6, 0);
  ftotal[2] += -9.8;
  BOOST_FOREACH(ContactPtr& contact, m_contacts) {
    DblVec fcontact = CalcWrenchValue(x, *contact);
    for (int j=0; j < 6; ++j) ftotal[j] += fcontact[j];
  }
  return ftotal;
}

ConvexConstraintsPtr ForceBalanceConstraint::convex(const vector<double>& x, Model* model) {
  AffExprVector ftotal(6);
  ftotal[2].constant += -9.8;
  BOOST_FOREACH(ContactPtr& contact, m_contacts) {
    AffExprVector fcontact = CalcWrenchExpr(x, *contact);
    exprInc(ftotal, fcontact);
  }
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  for (int i=0; i < 6; ++i) {
    out->addEqCnt(ftotal[i]);
  }
  return out;
}


struct SlidingFrictionErrorCalc : public VectorOfVector {
  // dofs cp ffr fn
  
  int m_nDof;
  double m_mu;
  Face m_face;
  KinBody::LinkPtr m_link;
  ConfigurationPtr m_config;
  
  SlidingFrictionErrorCalc(ConfigurationPtr config, double mu, const Face& face)
    : m_nDof(config->GetDOF()), m_mu(mu), m_face(face), m_link(face.link),m_config(config) {
    }
  VectorXd operator()(const VectorXd& x) const;
  
};

VectorXd SlidingFrictionErrorCalc::operator()(const VectorXd& x) const {
  VectorXd dofs0 = x.topRows(m_nDof);
  VectorXd dofs1 = x.middleRows(m_nDof, m_nDof);

  OR::Vector localp;
  localp[0] = x(2*m_nDof + 0);
  localp[1] = x(2*m_nDof + 1);

  double ffrx = x(2*m_nDof + 2 + 0);
  double ffry = x(2*m_nDof + 2 + 1);
  double fn = x(2*m_nDof + 2 + 2);
      
  m_config->SetDOFValues(toDblVec(x.middleRows(m_nDof, m_nDof)));
  OR::Vector p1world = m_link->GetTransform().trans;
  m_config->SetDOFValues(toDblVec(x.topRows(m_nDof)));
  OR::Vector p0world = m_link->GetTransform().trans;
  // pppp << p1world << " | " <<  p0world << endl;
  OR::Vector localvel0 = geometry::quatRotate(
    geometry::quatMultiply(geometry::quatInverse(m_face.Tlf.rot), 
                           geometry::quatInverse(m_link->GetTransform().rot)), 
              (p1world - p0world));
  double normvel = sqrtf(localvel0.lengthsqr2());
  // double normf = sqrtf(ffrx*ffrx + ffry*ffry);
  Vector2d err;
  err(0) = normvel * ffrx - m_mu * fn * localvel0[0] ;
  err(1) = normvel * ffry - m_mu * fn * localvel0[1] ;
  return err;  
}

struct QuadraticCost : public Cost {
  QuadExpr m_expr;
  QuadraticCost(const QuadExpr& expr) : m_expr(expr) {}
  double value(const DblVec& x) {return m_expr.value(x);}
  ConvexObjectivePtr convex(const DblVec& x, Model* model)  {
    ConvexObjectivePtr out(new ConvexObjective(model));
    exprInc(out->quad_, m_expr);
    return out;
  }
  
};

struct MechanicsProblem: public OptProb {
  int m_nSteps;
  typedef vector<ContactPtr> ContactVec;
  vector<ContactVec> m_timestep2contacts;

  RobotAndDOFPtr m_robotConfig;
  BasicArray<IncrementalRBPtr> m_dynObjConfigs;
  vector<StaticObjectPtr> m_staticObjConfigs;
  vector<KinBodyPtr> m_dynBodies, m_staticBodies;

  VarArray m_th; // robot
  vector<VarArray> m_obj2posvars; // position
  vector<MatrixXd> m_obj2q; // quaternion
  vector<VarArray> m_obj2r; // incremental rotation

  MechanicsProblem(int nSteps, const vector<KinBodyPtr>& dynamicBodies, const vector<KinBodyPtr>& staticBodies, RobotAndDOFPtr robotConfig);

  void AddContacts(ContactType ctype, int iFirst, int iLast, const Face& faceA, const Face& faceB);
  void AddMotionCosts(float jointvel_coeff, float obj_linvel_coeff, float obj_angvel_coeff);
  void AddDynamicsConstraints();
  void AddCollisionCosts(double coeff);
  ConfigurationPtr GetConfig(KinBody::LinkPtr, int timestep);
  VarVector GetDOFVars(KinBody::LinkPtr, int timestep);
  void Callback(DblVec&);
  void AnimateSolution(const DblVec& x);
  vector<ContactPtr> GetContacts(KinBody::LinkPtr link, int timestep);

private:
  void AddContact(int iStep, const Face& faceA, const Face& faceB, const VarVector& ptAvars, const VarVector& ptBvars, const Var& fn, const VarVector& ffr);
  void AddContactVariables(ContactType ctype, int iFirst, int iLast, const Face& faceA, const Face& faceB, VarArray& ptAVars, VarArray& ptBVars, VarArray& fnVars, VarArray& ffrVars);
  vector<ContactPtr> GetContacts(KinBody::LinkPtr);
};


vector<ContactPtr> MechanicsProblem::GetContacts(KinBody::LinkPtr link, int timestep) {
  vector<ContactPtr> out;
  BOOST_FOREACH(const ContactPtr& contact, m_timestep2contacts[timestep]) {
    if (contact->m_faceA.link == link || contact->m_faceB.link == link) out.push_back(contact);
  }
  return out;
}

ConfigurationPtr MechanicsProblem::GetConfig(KinBody::LinkPtr link, int timestep) {
  KinBodyPtr body = link->GetParent();
  if (GetLinkMaybeAttached(m_robotConfig->GetRobot(), link->GetName())) return m_robotConfig;
  for (int i=0; i < m_dynBodies.size(); ++i) if (m_dynBodies[i] == body) return m_dynObjConfigs(timestep, i);
  for (int i=0; i < m_staticBodies.size(); ++i) if (m_staticBodies[i] == body) return m_staticObjConfigs[i];
  return ConfigurationPtr();
}

VarVector MechanicsProblem::GetDOFVars(KinBody::LinkPtr link, int timestep) {
  KinBodyPtr body = link->GetParent();
  if (GetLinkMaybeAttached(m_robotConfig->GetRobot(), link->GetName())) return m_th.row(timestep);
  for (int i=0; i < m_dynBodies.size(); ++i) if (m_dynBodies[i] == body) return m_obj2posvars[i].row(timestep);
  for (int i=0; i < m_staticBodies.size(); ++i) if (m_staticBodies[i] == body) return VarVector();
  assert(0);
  return VarVector();
}


MechanicsProblem::MechanicsProblem(int nSteps, const vector<KinBodyPtr>& dynBodies, const vector<KinBodyPtr>& staticBodies, RobotAndDOFPtr robotConfig) :
    m_nSteps(nSteps),
    m_robotConfig(robotConfig),
    m_dynBodies(dynBodies),
    m_staticBodies(staticBodies)
{
  m_dynObjConfigs.resize(m_nSteps, dynBodies.size());
  for (int iStep = 0; iStep  < m_nSteps; ++iStep) {
    for (int iBody = 0; iBody < m_dynBodies.size(); ++iBody) {
      m_dynObjConfigs.at(iStep, iBody).reset(new IncrementalRB(m_dynBodies[iBody]));
    }
  }

  m_staticObjConfigs.resize(staticBodies.size());
  for (int iBody = 0; iBody < m_staticBodies.size(); ++iBody) {
    m_staticObjConfigs[iBody].reset(new StaticObject(m_staticBodies[iBody]));
  }

  m_timestep2contacts.resize(m_nSteps);

  m_obj2posvars.resize(dynBodies.size());
  m_obj2q.resize(dynBodies.size());
  m_obj2r.resize(dynBodies.size());

  vector<VarArray*> arrs;
  vector<string> prefixes;
  vector<int> cols;
  arrs += &m_th;
  prefixes += "th";
  cols += m_robotConfig->GetDOF();
  for (int i=0; i < dynBodies.size(); ++i) {
    arrs += &m_obj2posvars[i];
    prefixes += (boost::format("obj%i")%i).str();
    cols += 6;
  }
  AddVarArrays(*this, m_nSteps, cols, prefixes, arrs);


  for (int iBody=0; iBody < dynBodies.size(); ++iBody) {
    m_obj2r[iBody] = m_obj2posvars[iBody].block(0, 3, m_nSteps, 3);
    MatrixXd& q = m_obj2q[iBody];
    q.resize(m_nSteps,4);
    for (int iStep = 0; iStep < m_nSteps; ++iStep) {
      q.row(iStep) = Vector4d(1, 0, 0, 0).transpose();
    }
  }

}

void MechanicsProblem::AddContacts(ContactType ctype, int iFirst, int iLast, const Face& faceA, const Face& faceB) {
  VarArray ptAVars, ptBVars, fnVars, ffrVars;
  AddContactVariables(ctype, iFirst, iLast, faceA, faceB, ptAVars, ptBVars, fnVars, ffrVars);
  for (int iStep=iFirst; iStep <= iLast; ++iStep) {
    int iCP = (ctype == SLIDING) ? iStep : 0;
    AddContact(iStep, faceA, faceB, ptAVars.row(iCP), ptBVars.row(iCP), fnVars(iStep,0), ffrVars.row(iStep));
    if (ctype==SLIDING) {
      // addLinearConstraint(ffrVars(iStep,0));
      // addLinearConstraint(ffrVars(iStep,1));
    }
    
  }
  
  if (ctype == SLIDING) {
    for (int iStep=iFirst; iStep< iLast; ++iStep) {
      ConfigurationPtr configA = GetConfig(faceA.link, iStep);
      ConfigurationPtr configB = GetConfig(faceB.link, iStep);      
      ConfigurationPtr compositeConfig(new CompositeConfig(configA, configB));
      VarVector compositeDOFVars0 = concat(GetDOFVars(faceA.link, iStep), GetDOFVars(faceB.link, iStep));
      VarVector compositeDOFVars1 = concat(GetDOFVars(faceA.link, iStep+1), GetDOFVars(faceB.link, iStep+1));
      VarVector vars = concat(compositeDOFVars0, compositeDOFVars1, ptAVars.row(iStep), ffrVars.row(iStep), fnVars.row(iStep));
      VectorOfVectorPtr f(new SlidingFrictionErrorCalc(GetConfig(faceA.link, iStep), mu, faceA));
      addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, VectorXd::Ones(2), INEQ, "slipfric")));
    }    
  }

  QuadExpr forcesquared;
  for (int i=0; i < fnVars.size(); ++i) exprInc(forcesquared, exprSquare(fnVars.m_data[i]));
  addCost(CostPtr(new QuadraticCost(exprMult(forcesquared,.1))));
}


// after you've created contact point variables and constrained them to lie in the face
void MechanicsProblem::AddContact(int iStep, const Face& faceA, const Face& faceB, const VarVector& ptAVars, const VarVector& ptBVars, const Var& fn, const VarVector& ffr) {
  ConfigurationPtr configA = GetConfig(faceA.link, iStep);
  ConfigurationPtr configB = GetConfig(faceB.link, iStep);
  ConfigurationPtr compositeConfig(new CompositeConfig(configA, configB));
  VarVector compositeDOFVars = concat(GetDOFVars(faceA.link, iStep), GetDOFVars(faceB.link, iStep));
  ContactPtr contact(new Contact(SLIDING, compositeConfig, faceA, faceB, compositeDOFVars, ptAVars, ptBVars, fn, ffr));
  addConstraint(ConstraintPtr(new FaceContactConstraint(compositeConfig, compositeDOFVars, ptAVars, ptBVars, faceA, faceB)));
  m_timestep2contacts[iStep].push_back(contact);
}

void MechanicsProblem::AddContactVariables(ContactType ctype, int iFirst, int iLast, const Face& faceA, const Face& faceB, VarArray& ptAVars, VarArray& ptBVars, VarArray& fnVars, VarArray& ffrVars) {

  if (ctype == SLIDING) {
    vector<int> cols; cols += 1,2,2,2;
    vector<string> names; names += "fn", "ffr", "ptA", "ptB";
    vector<VarArray*> arrayPtrs; arrayPtrs += &fnVars, &ffrVars, &ptAVars, &ptBVars;
    AddVarArrays(*this, iLast - iFirst + 1, cols, names, arrayPtrs);
  }
  else {
    {
      vector<int> cols; cols += 1,2;
      vector<string> names; names += "fn", "ffr";
      vector<VarArray*> arrayPtrs; arrayPtrs += &fnVars, &ffrVars;
      AddVarArrays(*this, iLast - iFirst + 1, cols, names, arrayPtrs);
    }
    {
      vector<int> cols; cols += 2,2;
      vector<string> names; names += "ptA", "ptB";
      vector<VarArray*> arrayPtrs; arrayPtrs += &ptAVars, &ptBVars;
      AddVarArrays(*this, 1, cols, names, arrayPtrs);
    }
  }

  setLowerBounds(vector<double>(fnVars.size(), 0), fnVars.flatten());
  
  for (int i = iFirst; i <= iLast; ++i) {
    // TODO add quadratic friction cone constraint 
    // norm(f) < mu n
    // addLinearConstr(- mu * fnVars(i,0) + ffrVars(i,0), INEQ);
    // addLinearConstr(- mu * fnVars(i,0) - ffrVars(i,0), INEQ);
    // addLinearConstr(- mu * fnVars(i,0) + ffrVars(i,1), INEQ);
    // addLinearConstr(- mu * fnVars(i,0) - ffrVars(i,1), INEQ);
    getModel()->addIneqCnt(exprSquare(ffrVars(i,0)) + exprSquare(ffrVars(i,1)) - exprMult(exprSquare(fnVars(i,0)),mu), "fc");
  }
  
  VectorXd Ac, Bc;
  MatrixX2d Aab, Bab;
  PolygonToEquations(faceA.poly, Aab, Ac);
  PolygonToEquations(faceB.poly, Bab, Bc);
  int nContacts = ptAVars.rows();
  for (int i=0; i < nContacts; ++i) {
    for (int iSide = 0; iSide < Aab.rows(); ++iSide) {
      addLinearConstraint(Aab(iSide, 0) * ptAVars(i,0) + Aab(iSide, 1) * ptAVars(i,1) + Ac(iSide),INEQ);
    }
    for (int iSide = 0; iSide < Bab.rows(); ++iSide) {
      addLinearConstraint(Bab(iSide, 0) * ptBVars(i,0) + Bab(iSide, 1) * ptBVars(i,1) + Bc(iSide),INEQ);
    }
  }
  

}


void MechanicsProblem::AddMotionCosts(float jointvel_coeff, float obj_linvel_coeff, float obj_angvel_coeff) {
  addCost(CostPtr(new JointVelCost(m_th, jointvel_coeff * VectorXd::Ones(m_th.cols()))));
  for (int i=0; i < m_dynBodies.size(); ++i) {
    // xxx if this is the only usage of m_obj2r, then don't make it
    if (obj_linvel_coeff > 0) addCost(CostPtr(new JointVelCost(m_obj2posvars[i].block(0,0,m_nSteps,3), obj_linvel_coeff*VectorXd::Ones(3) )));
    if (obj_angvel_coeff > 0) addCost(CostPtr(new AngVelCost(m_dynObjConfigs.col(i), m_obj2r[i], obj_angvel_coeff )));
  }

}

void MechanicsProblem::AddCollisionCosts(double coeff) {
  // XXX assumes there's two of them
  for (int iStep=0; iStep < m_nSteps; ++iStep) {
    ConfigurationPtr compositeConfig(new CompositeConfig(m_robotConfig, m_dynObjConfigs(iStep,0)));
    VarVector compositeDOFVars = concat(m_th.row(iStep), m_obj2posvars[0].row(iStep));
    addCost(CostPtr(new CollisionCost(0, coeff, compositeConfig, compositeDOFVars)));
  }
}

void MechanicsProblem::AddDynamicsConstraints() {
  // for each non-robot object
  for (int iStep = 0; iStep < m_nSteps; ++iStep) {
    for (int iObj = 0; iObj < m_dynBodies.size(); ++iObj) {
      ConfigurationPtr config = m_dynObjConfigs(iStep, iObj);
      KinBody::LinkPtr link = m_dynBodies[iObj]->GetLinks()[0];
      VarVector dofvars = GetDOFVars(link, iStep);
      addConstraint(ConstraintPtr(new ForceBalanceConstraint(config, link, GetContacts(link, iStep), dofvars)));
    }
  }
}

void PlotFace(EnvironmentBase& env, const Face& face, vector<OR::GraphHandlePtr>& handles) {
  int npts = face.poly.rows();
  MatrixX3f ptdata(npts, 3);
  OR::Transform Twf = face.link->GetTransform()*face.Tlf;
  for (int i=0; i < npts; ++i) {
    OR::Vector ptlocal(face.poly(i,0), face.poly(i,1), 0);
    OR::Vector ptworld = Twf*ptlocal;
    for (int j=0; j < 3; ++j) ptdata(i,j) = ptworld[j];
  }
  handles.push_back(env.drawlinestrip(ptdata.data(), ptdata.rows(), sizeof(float)*3, 4, OR::RaveVector<float>(1,0,0,1)));

  Vector3f ecw = ptdata.colwise().mean();
  OR::Vector centerworld(ecw(0), ecw(1), ecw(2));
  OR::Vector normalface(0,0,1);
  OR::Vector normalworld = OR::geometry::quatRotate(Twf.rot, normalface);

  handles.push_back(env.drawarrow(centerworld, centerworld + .05*normalworld, .001, OR::RaveVector<float>(1,1,0,1)));
}

void MechanicsProblem::AnimateSolution(const DblVec& x) {
  EnvironmentBasePtr env = m_robotConfig->GetEnv();
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);

  for (int iStep=0; iStep < m_nSteps; ++iStep) {
    m_robotConfig->SetDOFValues(getDblVec(x, m_th.row(iStep)));
    for (int iObj=0; iObj < m_dynBodies.size(); ++iObj) {
      DblVec objdofs = getDblVec(x, m_obj2posvars[iObj].row(iStep));
      m_dynObjConfigs(iStep, iObj)->SetDOFValues(objdofs);
    }    
    printf("step %i\n",iStep);
    viewer->Idle();
  }

  
}

void MechanicsProblem::Callback(DblVec& x) {


  // Update quats
  for (int iObj=0; iObj < m_dynBodies.size(); ++iObj) {
    MatrixXd& q = m_obj2q[iObj];
    MatrixXd rvals = getTraj(x, m_obj2r[iObj]);
    for (int iStep = 0; iStep < q.rows(); ++iStep) {
      q.row(iStep) = quatMult(quatExp(rvals.row(iStep)), q.row(iStep));
      // cout << quatMult(quatExp(rvals.row(iStep)), q.row(iStep)).transpose() << " =?= " << geometry::quatMultiply(geometry::quatFromAxisAngle(toRave(rvals.row(iStep))), toRaveQuat(q.row(iStep))) << endl;
      // cout << quatExp(rvals.row(iStep)).transpose() << " =?= " << OR::geometry::quatFromAxisAngle(toRave(rvals.row(iStep))) << endl;
      m_dynObjConfigs(iStep, iObj)->m_q = toRaveQuat(q.row(iStep));
      m_dynObjConfigs(iStep, iObj)->m_r = OR::Vector(0,0,0);
    }
    setVec(x, m_obj2r[iObj].flatten(), DblVec(m_obj2r[iObj].size(), 0));
  }


  // Plotting
  RobotBasePtr robot = boost::dynamic_pointer_cast<RobotAndDOF>(m_robotConfig)->GetRobot();
  EnvironmentBasePtr env = robot->GetEnv();
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);

  assert(robot);

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
//  cout << "robotTraj\n" << robotTraj << endl;

  for (int iStep=0; iStep < m_nSteps; ++iStep) {
    m_robotConfig->SetDOFValues(toDblVec(robotTraj.row(iStep)));
    
    vector<ContactPtr> contacts = m_timestep2contacts[iStep];
    BOOST_FOREACH(ContactPtr contact, contacts) {
      if (contact->m_faceA.link->GetParent() == robot) PlotFace(*env, contact->m_faceA, handles);
      else if (contact->m_faceB.link->GetParent() == robot) PlotFace(*env, contact->m_faceB, handles);
    }
    
    handles.push_back(viewer->PlotKinBody(robot));
    SetTransparency(handles.back(), .35);
  }


  for (int iObj=0; iObj < m_dynBodies.size(); ++iObj) {
    TrajArray objTraj = getTraj(x, m_obj2posvars[iObj]);
//    cout << "objTraj\n" << objTraj << endl;
    KinBodyPtr body = m_dynBodies[iObj];
    KinBody::LinkPtr link = body->GetLinks()[0];
    for (int iStep = 0; iStep < m_nSteps; ++iStep) {
        
      m_dynObjConfigs(iStep, iObj)->SetDOFValues(toDblVec(objTraj.row(iStep)));
      
      vector<ContactPtr> contacts = GetContacts(link, iStep);
      BOOST_FOREACH(const ContactPtr& contact, contacts) {
        PlotFace(*env, contact->m_faceA, handles);
        PlotFace(*env, contact->m_faceB, handles);
      }

      handles.push_back(viewer->PlotKinBody(body));

      SetTransparency(handles.back(), .35);
    }
  }


  BOOST_FOREACH(const ConstraintPtr& cnt, getConstraints()) {
    if (cnt->name()=="slipfric") {
      // cout << "slipfric val: " << CSTR(cnt->value(x)) << endl;
    }
  }

  if (idle) viewer->Idle();

}


void Callback(OptProb* prob, DblVec& x) {
  dynamic_cast<MechanicsProblem*>(prob)->Callback(x);
}




void Setup3DOFPush(EnvironmentBasePtr env, boost::shared_ptr<MechanicsProblem>& prob, DblVec& xinit) {


  Face armtip, boxface, boxbottom, tabletop;//xxx
  env->Load(string(DATA_DIR) + "/3link_pushing.env.xml");

  RobotBasePtr robot = GetRobotByName(*env, "3DOFRobot");
  assert(robot);
  KinBodyPtr obj = GetBodyByName(*env, "box");
  assert(obj);
  KinBodyPtr table = GetBodyByName(*env, "table");
  assert(table);


  vector<int> robotDofInds; 
  robotDofInds += 0, 1, 2;
  RobotAndDOFPtr robotConfig(new RobotAndDOF(robot, robotDofInds));

  {
    boxface.poly.resize(4,2);
    boxface.poly << -.1, -.1, -.1, .1, .1, .1, .1, -.1;
    boxface.Tlf.trans.x = .1;
    boxface.Tlf.rot = OpenRAVE::geometry::quatFromAxisAngle(OpenRAVE::Vector(0, 1, 0), M_PI / 2);
    boxface.link = obj->GetLinks()[0];
  }
  {
    armtip.poly.resize(4,2);
    armtip.poly << -.1, -.1, -.1, .1, .1, .1, .1, -.1;
    armtip.poly *= .05;
    armtip.Tlf.trans.x = .08;
    armtip.Tlf.rot =  OpenRAVE::geometry::quatFromAxisAngle(OpenRAVE::Vector(0, 1, 0), M_PI / 2);
    armtip.link = robot->GetLinks()[3];
  }
  {
    boxbottom = boxface;
    boxbottom.Tlf.rot = OpenRAVE::geometry::quatFromAxisAngle(OpenRAVE::Vector(0, 1, 0), M_PI);    
    boxbottom.Tlf.trans = OR::Vector(0,0,-.1);
  }
  {
    tabletop.poly.resize(4,2);
    tabletop.poly << -1.0, -1.0,  -1.0, 1.0,  1.0, 1.0,  1.0, -1.0;
    tabletop.Tlf.trans.z = .1;
    tabletop.link = table->GetLinks()[0];
  }
  
  vector<KinBodyPtr> staticBodies, dynamicBodies;
  dynamicBodies.push_back(obj);
  staticBodies.push_back(table);

  prob.reset(new MechanicsProblem(nSteps, dynamicBodies, staticBodies, robotConfig));

//
  prob->AddContacts(STICKING, 0, nSteps-1, boxface, armtip);
  prob->AddContacts(SLIDING, 0, nSteps-1, boxbottom, tabletop);
  prob->AddDynamicsConstraints();
  prob->AddMotionCosts(10,10,10);
   prob->AddCollisionCosts(1);
  BasicTrustRegionSQP opt(prob);

  xinit = DblVec(prob->getNumVars(), 0);

  VectorXd startPt(6);
  startPt.topRows(3) = toVector3d(obj->GetTransform().trans);
  VectorXd endPt = startPt;
  endPt[0] += -.1;
  

  VarArray& posvars = prob->m_obj2posvars[0];
  for (int j = 0; j < 6; ++j) {
    setVec(xinit, posvars.col(j),VectorXd::LinSpaced(nSteps, startPt(j), endPt(j)));
    VectorXd ps = VectorXd::LinSpaced(nSteps, startPt(j), endPt(j));
    
    // for (int i=0; i < nSteps; ++i) {
    //   prob->addLinearConstr(exprSub(AffExpr(posvars(i, j)), ps(i)), EQ);
    // }
    prob->addLinearConstraint(exprSub(AffExpr(posvars(0, j)), startPt[j]), EQ);
    if (j < 3) 
      prob->addLinearConstraint(exprSub(AffExpr(posvars(nSteps - 1, j)), endPt[j]),EQ);
    else
      prob->addLinearConstraint(AffExpr(posvars(nSteps - 1, j)),EQ);
  }


  // // TODO disabling rotation since something screwy is going on
  // for (int iStep=0; iStep < nSteps; ++iStep) {
  //   for (int j=3; j < 6; ++j) {
  //     prob->addLinearConstr(AffExpr(prob->m_obj2posvars[0](iStep, j)), EQ);
  //   }
  // }

}

Face GetFace(KinBody::LinkPtr link, const OR::Vector& dir) {  
  Face f;
  OR::AABB bb = link->ComputeLocalAABB();
  OR::Vector extents = bb.extents;
  double lens[2];
  int i=-1;
  int sgn=0;
  for (int j=0; j < 3; ++j) if (fabs(dir[j]) > 1e-5) {
    i = j;
    sgn = dir[j];
  }
  assert(i != -1);
  f.Tlf.rot = geometry::quatRotateDirection(OR::Vector(0,0,1), dir);
  cout << "dir: " << dir << "tlf.rot: " << f.Tlf.rot << endl;
  OR::Vector fextents = f.Tlf.inverse()*extents;
  
  f.Tlf.trans = bb.pos;
  f.Tlf.trans[i] += sgn*extents[i];

  lens[0] = fextents[0];
  lens[1] = fextents[1];

  f.poly.resize(4,2);
  f.poly << lens[0], lens[1], -lens[0], lens[1], -lens[0], -lens[1], lens[0], -lens[1];
  f.link = link;
  return f;
}

void SetupPR2Push(EnvironmentBasePtr env, boost::shared_ptr<MechanicsProblem>& prob, DblVec& xinit) {


  env->Load(string(DATA_DIR) + "/pr2_pushing.env.xml");

  RobotBasePtr robot = GetRobotByName(*env, "pr2");  
  
  KinBodyPtr box = GetBodyByName(*env, "box");
  KinBodyPtr table = GetBodyByName(*env, "table");
  assert(robot && box && table);

  RobotAndDOFPtr robotConfig(new RobotAndDOF(robot, GetManipulatorByName(*robot,"rightarm")->GetArmIndices()));

  KinBody::LinkPtr fingerlink0 = robot->GetLink("r_gripper_l_finger_tip_link");
  KinBody::LinkPtr fingerlink1 = robot->GetLink("r_gripper_r_finger_tip_link");
  
  KinBodyPtr fingerbox0 = GetBodyByName(*env, "fingerbox0");
  KinBodyPtr fingerbox1 = GetBodyByName(*env, "fingerbox1");
  fingerbox0->SetTransform(fingerlink0->GetTransform());
  fingerbox1->SetTransform(fingerlink1->GetTransform());
  robot->Grab(fingerbox0, fingerlink0);
  robot->Grab(fingerbox1, fingerlink1);    
  
  
  Face finger0 = GetFace(fingerbox0->GetLinks()[0], OR::Vector(1,0,0));
  Face finger1 = GetFace(fingerbox1->GetLinks()[0], OR::Vector(1,0,0));
  // finger0.poly /= 2;
  // finger1.poly /= 2;
    
  cout << "finger0 poly" << finger0.poly << endl;
    
  Face boxfront = GetFace(box->GetLinks()[0], OR::Vector(-1,0,0));
  Face boxbottom = GetFace(box->GetLinks()[0], OR::Vector(0,0,-1));
  Face tabletop = GetFace(table->GetLinks()[0], OR::Vector(0,0,1));

  vector<KinBodyPtr> staticBodies = list_of(table), dynamicBodies = list_of(box);

  prob.reset(new MechanicsProblem(nSteps, dynamicBodies, staticBodies, robotConfig));

  prob->AddContacts(STICKING, 0, nSteps-1, boxfront, finger0);
  prob->AddContacts(STICKING, 0, nSteps-1, boxfront, finger1);
  prob->AddContacts(SLIDING, 0, nSteps-1, boxbottom, tabletop);
  prob->AddDynamicsConstraints();
  prob->AddMotionCosts(10,10,10);
   prob->AddCollisionCosts(200);
  BasicTrustRegionSQP opt(prob);

  xinit = DblVec(prob->getNumVars(), 0);
  VectorXd startPt = VectorXd::Zero(6);
  startPt.topRows(3) = toVector3d(box->GetTransform().trans);
  VectorXd endPt = startPt;
  endPt[0] += .12;
  // endPt[1] = .05;

  cout << "box start: " << startPt.transpose() << endl;
  cout << "table transform" << table->GetLinks()[0]->GetTransform() << endl;


  DblVec dofvals = robotConfig->GetDOFValues();
  for (int j=0; j < robotConfig->GetDOF(); ++j) {
    setVec(xinit, prob->m_th.col(j), VectorXd::Ones(nSteps)*dofvals[j]);    
  }
  prob->Callback(xinit);

  

  VarArray& posvars = prob->m_obj2posvars[0];
  for (int i=0; i < nSteps; ++i) {
    for (int j=0; j < 6; ++j) {
      double ptarg = (startPt(j) * (nSteps-1-i) + endPt(j) * i) / (nSteps-1);
      xinit[posvars(i,j).var_rep->index] = ptarg;
      if (constrain_all_poses || (i==0) || (i==nSteps-1)) prob->addLinearConstraint(exprSub(AffExpr(posvars(i, j)), ptarg), EQ);
    }
  }
  
}

void SetupPR2Lift(EnvironmentBasePtr env, boost::shared_ptr<MechanicsProblem>& prob, DblVec& xinit) {


  env->Load(string(DATA_DIR) + "/pr2_lifting.env.xml");

  RobotBasePtr robot = GetRobotByName(*env, "pr2");  
  
  KinBodyPtr box = GetBodyByName(*env, "box");
  KinBodyPtr table = GetBodyByName(*env, "table");
  assert(robot && box && table);
  
  vector<int> arminds = concat( GetManipulatorByName(*robot,"rightarm")->GetArmIndices(),
                                GetManipulatorByName(*robot,"leftarm")->GetArmIndices());

  RobotAndDOFPtr robotConfig(new RobotAndDOF(robot, arminds));

  KinBody::LinkPtr rfingerlink0 = robot->GetLink("r_gripper_l_finger_tip_link");
  KinBody::LinkPtr rfingerlink1 = robot->GetLink("r_gripper_r_finger_tip_link");
  KinBody::LinkPtr lfingerlink0 = robot->GetLink("l_gripper_l_finger_tip_link");
  KinBody::LinkPtr lfingerlink1 = robot->GetLink("l_gripper_r_finger_tip_link");
  
  KinBodyPtr rfingerbox0 = GetBodyByName(*env, "rfingerbox0");
  KinBodyPtr rfingerbox1 = GetBodyByName(*env, "rfingerbox1");
  KinBodyPtr lfingerbox0 = GetBodyByName(*env, "lfingerbox0");
  KinBodyPtr lfingerbox1 = GetBodyByName(*env, "lfingerbox1");
  
  rfingerbox0->SetTransform(rfingerlink0->GetTransform());
  rfingerbox1->SetTransform(rfingerlink1->GetTransform());
  robot->Grab(rfingerbox0, rfingerlink0);
  robot->Grab(rfingerbox1, rfingerlink1);    

  lfingerbox0->SetTransform(lfingerlink0->GetTransform());
  lfingerbox1->SetTransform(lfingerlink1->GetTransform());
  robot->Grab(lfingerbox0, lfingerlink0);
  robot->Grab(lfingerbox1, lfingerlink1);    
  
  
  Face rfinger0 = GetFace(rfingerbox0->GetLinks()[0], OR::Vector(1,0,0));
  Face rfinger1 = GetFace(rfingerbox1->GetLinks()[0], OR::Vector(1,0,0));
  Face lfinger0 = GetFace(lfingerbox0->GetLinks()[0], OR::Vector(1,0,0));
  Face lfinger1 = GetFace(lfingerbox1->GetLinks()[0], OR::Vector(1,0,0));
  // finger0.poly /= 2;
  // finger1.poly /= 2;
    
  Face boxleft = GetFace(box->GetLinks()[0], OR::Vector(0,1,0));
  Face boxright = GetFace(box->GetLinks()[0], OR::Vector(0,-1,0));

  vector<KinBodyPtr> staticBodies = list_of(table), dynamicBodies = list_of(box);

  prob.reset(new MechanicsProblem(nSteps, dynamicBodies, staticBodies, robotConfig));

  prob->AddContacts(STICKING, 0, nSteps-1, boxleft, lfinger0);
  prob->AddContacts(STICKING, 0, nSteps-1, boxleft, lfinger1);
  prob->AddContacts(STICKING, 0, nSteps-1, boxright, rfinger0);
  prob->AddContacts(STICKING, 0, nSteps-1, boxright, rfinger1);
  prob->AddDynamicsConstraints();
  prob->AddMotionCosts(10,10,10);
   prob->AddCollisionCosts(200);
  BasicTrustRegionSQP opt(prob);

  xinit = DblVec(prob->getNumVars(), 0);
  VectorXd startPt = VectorXd::Zero(6);
  startPt.topRows(3) = toVector3d(box->GetTransform().trans);
  VectorXd endPt = startPt;
  endPt[2] += .1;

  cout << "box start: " << startPt.transpose() << endl;
  cout << "table transform" << table->GetLinks()[0]->GetTransform() << endl;

  DblVec dofvals = robotConfig->GetDOFValues();
  for (int j=0; j < robotConfig->GetDOF(); ++j) {
    setVec(xinit, prob->m_th.col(j), VectorXd::Ones(nSteps)*dofvals[j]);    
  }
  

  VarArray& posvars = prob->m_obj2posvars[0];
  for (int i=0; i < nSteps; ++i) {
    for (int j=0; j < 6; ++j) {
      double ptarg = (startPt(j) * (nSteps-1-i) + endPt(j) * i) / (nSteps-1);
      xinit[posvars(i,j).var_rep->index] = ptarg;
      if (constrain_all_poses || (i==0) || (i==nSteps-1)) prob->addLinearConstraint(exprSub(AffExpr(posvars(i, j)), ptarg), EQ);
    }
  }
  
}



#if 0
void SetupPR2Pickup(EnvironmentBasePtr env, boost::shared_ptr<MechanicsProblem>& prob, DblVec& xinit) {


  env->Load(string(DATA_DIR) + "/pr2_pushing.env.xml");

  RobotBasePtr robot = GetRobotByName(*env, "pr2");
  KinBodyPtr box = GetBodyByName(*env, "box");
  KinBodyPtr table = GetBodyByName(*env, "table");
  assert(robot && box && table);

  int nSteps = 10;

  RobotAndDOFPtr robotConfig(new RobotAndDOF(robot, GetManipulatorByName(*robot,"rightarm")->GetArmIndices()));


  Face finger0 = GetFace(robot->GetLink("r_gripper_l_finger_tip_link"), OR::Vector(1,0,0));
  Face finger1 = GetFace(robot->GetLink("r_gripper_r_finger_tip_link"), OR::Vector(1,0,0));

  Face boxfront = GetFace(box->GetLinks()[0], OR::Vector(-1,0,0));
  Face boxbottom = GetFace(box->GetLinks()[0], OR::Vector(0,0,-1));
  Face tabletop = GetFace(table->GetLinks()[0], OR::Vector(0,0,1));

  vector<KinBodyPtr> staticBodies = list_of(table), dynamicBodies = list_of(box);

  prob.reset(new MechanicsProblem(nSteps, dynamicBodies, staticBodies, robotConfig));

  prob->AddContacts(STICKING, 0, nSteps-1, finger0, boxfront);
  prob->AddContacts(STICKING, 0, nSteps-1, finger1, boxfront);
  prob->AddContacts(SLIDING, 0, nSteps-1, boxbottom, tabletop);
  prob->AddDynamicsConstraints();
  prob->AddMotionCosts(10,10,10);
   prob->AddCollisionCosts(1);
  BasicTrustRegionSQP opt(prob);

  xinit = DblVec(prob->getNumVars(), 0);
  VectorXd startPt(6);
  startPt.topRows(3) = toVector3d(box->GetTransform().trans);
  VectorXd endPt = startPt;
  endPt[0] += .1;

  VarArray& posvars = prob->m_obj2posvars[0];
  for (int i=0; i < nSteps; ++i) {
    for (int j=0; j < 6; ++j) {
      double ptarg = (startPt(j) * (nSteps-i) + endPt(j) * i) / (nSteps-1);
      xinit[posvars(i,j).var_rep->index] = ptarg;
      if (constrain_all_poses || (i==0) || (i==nSteps-1)) prob->addLinearConstraint(exprSub(AffExpr(posvars(i, j)), ptarg), EQ);
    }
  }

}
#endif

/**
Problems:
1. slide plate towards robot
2. lift a box
3. push a rolling chair
4. tip a book out of a bookshelf
5. grasp an object with a three-fingered hand

4. is interesting. Just initialize it with a contact between its finger and the top. and tell the optimizer it needs to slide the thing out.
so it'll first just try to push down and slide it. Let's first have a maybe top contact, and then we have two maybe side contacts. 

Assuming one of the contacting objects is a face/surface, we can use that

So each contact has a surface face. Contact knows how to calculate the world wrench, and the expression for it.
But I decided that it was better to do the force balance in the link frame so everything isn't dependent on the origin.
Or we could compromise by doing force balance in a world-aligned frame centered at the link COM.

So next generation: fix force balance, contact knows about which face to use
Then a later generation will accommodate non-faces


*/




int main(int argc, char** argv) {

  vector<string> validProblems; validProblems += "3linkpush", "pr2push", "pr2lift";
  string problem = validProblems[0];
  
  Config config;
  config.add(new Parameter<double>("mu", &mu, "mu (friction coefficient)"));
  config.add(new Parameter<string>("problem", &problem, "problem: [3linkpush, pr2push]"));
  config.add(new Parameter<bool>("idle", &idle, "idle after stuff"));
  config.add(new Parameter<bool>("constrain_all_poses", &constrain_all_poses, "constrain pose trajectory, as opposed to just start and end"));
  config.add(new Parameter<int>("nSteps", &nSteps, "number of timesteps"));
  CommandParser parser(config);
  parser.read(argc, argv);


  if (std::find(validProblems.begin(), validProblems.end(), problem) == validProblems.end()) {
    PRINT_AND_THROW("invalid problem name");    
  }

  RaveInitialize(false, Level_Debug);
  OpenRAVE::EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();

  
  DblVec xinit;
  boost::shared_ptr<MechanicsProblem> prob;
  
  if (problem == "3linkpush") {
    Setup3DOFPush(env, prob, xinit);    
  }
  else if (problem == "pr2push") {
    SetupPR2Push(env, prob, xinit);
  }
  else if (problem == "pr2lift") {
    SetupPR2Lift(env, prob, xinit);
  }
  
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);
  
  BasicTrustRegionSQP opt(prob);
  opt.initialize(xinit);
  opt.max_iter_ = 1000;
  opt.addCallback(&Callback);
  
  opt.optimize();
  prob->AnimateSolution(opt.x());

  RaveDestroy();
}
