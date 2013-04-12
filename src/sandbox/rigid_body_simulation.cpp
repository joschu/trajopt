#include <openrave-core.h>
#include <openrave/openrave.h>

#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "sco/expr_op_overloads.hpp"
#include "sco/optimizers.hpp"
#include "quat_ops.hpp"
#include <boost/assign.hpp>
using namespace boost::assign;
using namespace OpenRAVE;
using namespace trajopt;
using namespace sco;
using namespace Eigen;
using namespace std;



////// Helper functions ///////////


#include "dynamics_utils.hpp"

//////////////////////////////////////////////

struct ContactTraj {
  double m_dt;
  VarArray m_pA; // local contact point in A
  VarArray m_pB; // local contact point in B
  VarArray m_f; // force from B to A
  VarArray m_fn; // normal force magnitude
  ContactTraj(OptProb& prob, int n_steps, double dt);
};
typedef boost::shared_ptr<ContactTraj> ContactTrajPtr;

struct RigidBodyTraj {
  double m_dt;
  VarArray m_p; // position trajectory
  MatrixXd m_q; // quaternion trajectory. updated in callback
  VarArray m_r; // rotation increments
  AffArray m_v; // velocity
  AffArray m_a; // acceleration
  AffArray m_w; // angular velocity. updated in callback.
  AffArray m_aa; // angular acceleration. update in callback
  CntArray m_dynamicscnts;
  RigidBodyTraj(OptProb& prob, int n_steps, double dt);
  void UpdateRotationExpressions(const DblVec& x);
  void UpdateDynamicsConstraints();
};
typedef boost::shared_ptr<RigidBodyTraj> RigidBodyTrajPtr;


class RigidBodySim : public OptProb {
public:

  double m_dt; // timestep
  vector<RigidBodyTrajPtr> m_rbts;
  vector<ContactTrajPtr> m_cts;
  vector<KinBody::LinkPtr> m_links;
  RigidBodySim(const vector<KinBody::LinkPtr>& links, double dt);
  /**
   * Update integration constraints
   * Update rotation expressions
   */
  void Callback(DblVec& x);

};

void AddInsideConstraints(OptProb& prob, const VarArray& localpts, KinBody::LinkPtr link) {
  assert (link->GetGeometries().size() == 1);
  KinBody::Link::GeometryPtr geom = link->GetGeometries()[0];
  vector<AffExpr> ineqs;
  ModelPtr model = prob.getModel();
  switch (geom->GetType()) {
  case KinBody::Link::GEOMPROPERTIES::GeomBox:
    OpenRAVE::Vector halfExtents = geom->GetBoxExtents();
    VarVector vars;
    DblVec lbs, ubs;
    for (int i=0; i < localpts.rows(); ++i) {
      for (int j=0; j < localpts.cols(); ++j) {
        vars.push_back(localpts(i,j));
        lbs.push_back(-halfExtents[j]);
        ubs.push_back(halfExtents[j]);
      }
    }
    prob.setUpperBounds(vars, ubs);
    prob.setLowerBoudns(vars, lbs);
    break;
  default:
    PRINT_AND_THROW("only boxes are implemented");
}





class ContactComplementarity : public EqConstraint {
  VarVector m_originA, m_localA, m_rotIncA, m_originB, m_localB, m_rotIncB;
  Eigen::Map<const Vector3d> m_rotationA, m_rotationB;
  Var m_fn;

  ContactComplementarity(const VarVector& originA, const VarVector& localA, const VarVector& rotIncA, const Vector4d& rotationA,
                         const VarVector& originB, const VarVector& localB, const VarVector& rotIncB, const Vector4d& rotationB,
                        const Var& fn) :
      m_originA(originA), m_localA(localA), m_rotIncA(rotIncA), m_rotationA(rotationA),
      m_originB(originB), m_localB(localB), m_rotIncB(rotIncB), m_rotationB(rotationB),
      m_fn(fn) {}
                                    
  vector<double> value(const vector<double>& x);
  ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
};

vector<double> ContactComplementarity::value(const vector<double>& x) {

  Vector3d val_originA = getVec(x, m_originA);
  Vector3d val_localA = getVec(x, m_localA);
  Vector3d val_rotIncA = getVec(x, m_rotIncA);
  Vector4d newRotationA = quatMult(quatExp(val_rotIncA), m_rotationA);
  Vector3d newPositionWorldA = val_originA + quatRotate(newRotationA, val_localA);

  Vector3d val_localB = getVec(x, m_localB);
  Vector3d val_originB = getVec(x, m_originB);    
  Vector3d val_rotIncB = getVec(x, m_rotIncB);    
  Vector4d newRotationB = quatMult(quatExp(val_rotIncB), m_rotationB);
  Vector3d newPositionWorldB = val_originB + quatRotate(newRotationB, val_localB);
  
  val_fn = m_fn.value(x);
  
  return toDblVec(newPositionWorldA - newPositionWorldB) * val_fn;
}

ConvexConstraintsPtr ContactComplementarity::convex(const vector<double>& x, Model* model) {


  ///////// verbatim from value()
  Vector3d val_originA = getVec(x, m_originA);
  Vector3d val_localA = getVec(x, m_localA);
  Vector3d val_rotIncA = getVec(x, m_rotIncA);
  Vector4d newRotationA = quatMult(quatExp(val_rotIncA), m_rotationA);
  Vector3d newPositionWorldA = val_originA + quatRotate(newRotationA, val_localA);

  Vector3d val_localB = getVec(x, m_localB);
  Vector3d val_originB = getVec(x, m_originB);    
  Vector3d val_rotIncB = getVec(x, m_rotIncB);    
  Vector4d newRotationB = quatMult(quatExp(val_rotIncB), m_rotationB);
  Vector3d newPositionWorldB = val_originB + quatRotate(newRotationB, val_localB);
      
  val_fn = m_fn.value(x);
  ///////////
      
  AffExprVector newPositionWorldExprA(3);
  exprVecInc(newPositionWorldExprA, m_originA);
  exprVecInc(newPositionWorldExprA, quatRotateExpr(m_rotationA, m_localA));
  exprVecInc(newPositionWorldExprA, exprCross(m_rotIncA, exprVecSub(newPositionWorldA, val_originA)));

  AffExprVector newPositionWorldExprB(3);
  exprVecInc(newPositionWorldExprB, m_originB);
  exprVecInc(newPositionWorldExprB, quatRotateExpr(m_rotationB, m_localB));
  exprVecInc(newPositionWorldExprB, exprCross(m_rotIncB, exprVecSub(newPositionWorldB, val_originB)));
      
  VectorXd curdiff = newPositionWorldA - newPositionWorldB;
  AffExprVector exprdiff = exprVecSub(newPositionWorldExprA, newPositionWorldExprB);
  
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  for (int i=0; i < 3; ++i) {
    AffExpr fd;
    exprInc(fd, curdiff[i] * m_fn); // diff wrt contact points
    exprInc(fd, exprdiff[i] * val_fn); // diff wrt force
    exprDec(fd, curdiff[i] * val_fn); // double counted it
    out->addEqCnt(fd);
  }
  return out;
}



ContactTraj::ContactTraj(RigidBodyTrajPtr trajA, RigidBodyTrajPtr trajB, OptProb& prob, int n_steps, double dt) : m_dt(dt) {
  vector<VarArray*> arrs; arrs += m_pA, m_pB, m_f;
  vector<string> prefixes; prefixes += "pA", "pB", "f", "fn";
  vector<int> cols; cols += 3,3,3,1;
  AddVarArrays(prob, n_steps, cols, prefixes, arrs);
  AddInsideConstraints(prob, m_pA, trajA->m_link);
  AddInsideConstraints(prob, m_pB, trajB->m_link);

  for (int i=0; i < n_steps; ++i) {
    prob.addConstr(ConstraintPtr(new ContactComplementarity(trajA->m_x.row(i), m_pA.row(i), trajA->m_r.row(i), trajA->m_q.row(i),
                                                            trajB->m_x.row(i), m_pB.row(i), trajB->m_r.row(i), trajB->m_q.row(i))));  
  }
}

RigidBodyTraj::RigidBodyTraj(OptProb& prob, KinBody::LinkPtr link, int n_steps, int dt) : m_link(link), m_dt(dt) {
  vector<VarArray*> arrs; arrs += m_p, m_r;
  vector<string> prefixes; prefixes += "p", "r";
  vector<int> cols; cols += 3,3;
  AddVarArrays(prob, n_steps, cols, prefixes, arrs);
  FiniteDifferences(m_x, m_v, m_dt);
  FiniteDifferences(m_v, m_a, m_dt);
}

void RigidBodyTraj::UpdateRotationExpressions(const DblVec& x) {
  MatrixXd rvals = getTraj(x, m_r);
  // update quaternions
  for (int i=0; i < m_q.rows(); ++i) {
    m_q.row(i) = quatMult(quatExp(rvals.row(i)), m_q.row(i));
  }

  MatrixXd wvals = getW(m_q, m_dt); // angular velocity from previous trajectory
  for (int i=0; i < wvals.rows(); ++i) {
    for (int j=0; j < wvals.cols(); ++j) {
      m_w(i,j) = (m_r(i+1,j) - m_r(i,j))/m_dt + wvals(i,j);  // add derivative of increments
    }
  }
  FiniteDifferences(m_w, m_aa, m_dt);

}

void RigidBodySim::Callback(DblVec& x) {
  // update rotation expressions
  // set up integration constraints
  for (int i=0; i < m_rbts.size(); ++i) m_rbts[i]->UpdateRotationExpressions(x);
}

RigidBodySim::RigidBodySim(const vector<KinBody::LinkPtr>& links, int n_steps, double dt) : m_links(links), m_dt(dt) {
  for (int i=0; i < links.size(); ++i) {
    m_rbts.push_back(RigidBodyTrajPtr(new RigidBodyTraj(*this, links[i], n_steps, dt)));
  }
  for (int i=0; i < links.size(); ++i) {
    for (int j=0; j < links.size(); ++j) {
      m_cts.push_back(ContactTrajPtr(new ContactTraj(links[i], links[j], *this, n_steps, dt)));
    }
  }
}


void Callback(OptProb* prob, DblVec& x) {
  dynamic_cast<RigidBodySim*>(prob)->Callback(x);
}


/**
class TRAJOPT_API Configuration {
  virtual void SetDOFValues(const DblVec& dofs) = 0;
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const = 0;
  virtual DblVec GetDOFValues() = 0;
  virtual int GetDOF() const = 0;
  virtual EnvironmentBasePtr GetEnv() = 0;
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const = 0;
  virtual DblMatrix RotationJacobian(int link_ind) const = 0;
  virtual bool DoesAffect(const KinBody::Link& link) = 0;
  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() = 0;
  virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) = 0;
  virtual DblVec RandomDOFValues() = 0;  
};
*/

class RigidBody : public Configuration {
  KinBody::LinkPtr m_link;
  Vector3d m_x;
  Vector3d m_r;
  Vector4d m_q;  
  BodyIncRot(const KinBody::LinkPtr link, const Vector4d& q) : m_link(link), m_q(q) {}
  void SetQ(const Vector4d q) {
    m_q = q;
  }
  virtual void SetDOFValues(const DblVec& dofs) = 0;
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const {
    lower.resize(6);
    upper.resize(6);
    for (int i=0; i < 3; ++i) {
      lower(i) = -INFINITY;
      upper(i) = INFINITY;
      lower(i+3) = -1;
      upper(i+3) = 1;
    }
  }
  virtual DblVec GetDOFValues() {
    DblVec out(6);
    out[0] = m_x(0);
    out[1] = m_x(1);
    out[2] = m_x(2);
    out[3] = m_r(3);
    out[4] = m_r(4);
    out[5] = m_r(5);
  };
  virtual int GetDOF() const {return 6;}
  virtual EnvironmentBasePtr GetEnv() {return m_link->GetParent()->GetEnv();};
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const {
    DblMatrix out(3,6);
    out.leftCols(3) = Matrix3d::Identity();
    out.rightCols(3) = antiSym(pt - m_x);
  }
  virtual DblMatrix RotationJacobian(int link_ind) {PRINT_AND_THROW("not implemented");};
  virtual bool DoesAffect(const KinBody::Link& link) {
    return link == m_link;
  }
  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() {
    return std::vector<KinBody::LinkPtr> out(1, m_link);
  }
  virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) {
    links = GetAffectedLinks();
    link_inds = vector<int>(1,0);
  }
  virtual DblVec RandomDOFValues() {
    PRINT_AND_THROW("not implemented");
  }
};


class CompositeConfig : public Configuration {
public:
  
  vector<ConfigurationPtr> m_configs;
  
  virtual void SetDOFValues(const DblVec& dofs) {
    int n=0;
    BOOST_FOREACH(ConfigPtr& config, m_configs) {
      DblVec part_dofs(dofs.begin()+n, dofs.begin()+n+config->GetDOF());
      config->SetDOFValues(part_dofs);      
      n+=config->GetDOF()
    }
  }
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const {
    int n=0;
    lower.clear();
    upper.clear();
    BOOST_FOREACH(ConfigPtr& config, m_configs) {
      DblVec part_lower, part_upper;
      config->GetDOFLimits(part_lower, part_upper);
      lower.insert(part_lower.begin(), part_lower.end());
      upper.insert(part_upper.begin(), part_upper.end());
    }
    
  }
  virtual DblVec GetDOFValues() {
    DblVec out;
    BOOST_FOREACH(ConfigPtr& config, m_configs) {
      DblVec part_vals = config->GetDOFValues();
      out.insert(part_vals.begin(), part_vals.end());
    }        
    return out;
  }
  virtual int GetDOF() const {
    int out=0;
    BOOST_FOREACH(ConfigPtr& config, m_configs) {
      out += config->GetDOF();
    }    
    return out;    
  }
  virtual OpenRAVE::EnvironmentBasePtr GetEnv() {return m_configs[0]->GetEnv();}
  
  
  static const int PART_FACTOR = 1024;
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const {
    int i_part = link_ind / PART_FACTOR;
    DblMatrix out(3, GetDOF());
    int i_startDof = m_startInds[i_part];
    out.block(0, i_startDof, 3, m_configs[i_part]->GetDOF()) = m_configs[i_part]->PositionJacobian(link_ind%PART_FACTOR, pt);
  }
  virtual DblMatrix RotationJacobian(int link_ind) const {PRINT_AND_THROW("not implemented");}
  virtual bool DoesAffect(const KinBody::Link& link) {
    bool out = false;
    BOOST_FOREACH(ConfigPtr& config, m_configs) {
      out |= config->DoesAffect(link);
    }
    return out;
  };
  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() {
    std::vector<KinBody::LinkPtr> out;
    BOOST_FOREACH(ConfigPtr& config, m_configs) {
      std::vector<KinBody::LinkPtr> part_links = config->GetAffectedLinks();
      out.insert(part_links.begin(), part_links.end());
    }
    return out;
    
  }
  virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) {
    links.clear();
    link_inds.clear();
    i_part = 0;
    BOOST_FOREACH(ConfigPtr& config, m_configs) {
      std::vector<KinBody::LinkPtr> part_links;
      vector<int> part_link_inds;
      config->GetAffectedLinks(part_links, only_with_geom, part_link_inds);
      BOOST_FOREACH(int& i, part_link_inds) i += PART_FACTOR*i_part;
      links.insert(part_links.begin(), part_links.end());
      link_inds.insert(part_link_inds.begin(), part_link_inds.end());
      ++i_part;
    }
    
  }
  virtual DblVec RandomDOFValues() {
    PRINT_AND_THROW("not implemented");
  }
  
  struct CompositeSaver : public Saver {
    vector<SaverPtr> savers;
    CompositeSaver(CompositeConfig* self) {
      savers.reserve(self->m_configs.size());
      for (int i=0; i < self->m_configs.size(); ++i) {
        savers.push_back(self->m_configs[i]->Save());
      }
    }
  };
  SaverPtr Save() {
    return SaverPtr(new CompositeSaver(this));
  }
      
};
/**


Rigid body
------------

variables:
- has a set of contacts
- Integration constraints


Contact
---------

variables:
- local points
- normal force
- friction force vector
- complementarity constraints
- friction cone constraint
- friction direction constraint: fn * v - ffr * normv_aux. normv_aux <= norm(v)


Other
------
- non-overlap constraints

I should write a new system for getting rid of collisions
What's the abstraction to put around rigid bodies? At each timestep we have a different Configuration. On update, we set the base quaternion.

Let's set things up so we can test one thing at a time.
First just rigid bodies and robot with collisions
Let's code up all the tests first.

 * todo:
 * - inside constraints OK
 * - overlap constraints
 * - complementarity constraints OK
 * - set increment mask
 * - integration constraints
 * - friction cone constraints
 * - initializer with fwd sim?
 * - implement cones in gurobi

shit i just realized that my scheme doesn't set quaternions at different time steps.

Should I reimplement collision stuff so it's just a constraint on all pairs?
Or should I screw around with robotanddof

Problems with current scheme:
- should be able to make contact with static object
- should be able to make contact with robot
 */

int main(int argc, char** argv) {

  int n_steps = 10;
  double dt=.1;



  OptProbPtr prob(new RigidBodySim(xxx));
  BasicTrustRegionSQP opt(prob);

  opt.addCallback(&Callback);
  OptStatus status = opt.optimize();


}
