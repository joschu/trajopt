#pragma once
#include "trajopt/common.hpp"
#include "utils/eigen_conversions.hpp"
#include <iostream>

namespace trajopt {
struct IncrementalRB: public Configuration {
  OpenRAVE::KinBodyPtr m_body;
  OpenRAVE::Vector m_r;
  OpenRAVE::Vector m_q;

  IncrementalRB(OpenRAVE::KinBodyPtr body) :
      m_body(body), m_r(0, 0, 0), m_q(1,0,0,0) {
  }
  virtual void SetDOFValues(const DblVec& dofs) {
    OR::Transform T;
    T.trans = OR::Vector(dofs[0], dofs[1], dofs[2]);
    m_r = OR::Vector(dofs[3],dofs[4], dofs[5]);
    T.rot = OpenRAVE::geometry::quatMultiply(OpenRAVE::geometry::quatFromAxisAngle(m_r), m_q);
    m_body->SetTransform(T);
  }
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const {
    lower = DblVec(6, -INFINITY);
    upper = DblVec(6, INFINITY);
  };
  virtual DblVec GetDOFValues() {
    throw;
    DblVec out(6);
    OpenRAVE::Transform T = m_body->GetTransform();
    out[0] = T.trans.x;
    out[1] = T.trans.y;
    out[2] = T.trans.z;
    out[3] = m_r[0];
    out[4] = m_r[1];
    out[5] = m_r[2];
    return out;
  }
  ;
  virtual int GetDOF() const {
    return 6;
  }
  virtual OpenRAVE::EnvironmentBasePtr GetEnv() {
    return m_body->GetEnv();
  }
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const {
    MatrixXd out(3, 6);
    out.leftCols(3) = Matrix3d::Identity();
    assert(link_ind == 0);
    KinBody::LinkPtr link = m_body->GetLinks()[link_ind];
    OpenRAVE::Vector dr = pt - link->GetTransform().trans;
    double matdata[9] = { 0, dr[2], -dr[1], -dr[2], 0, dr[0], dr[1], -dr[0], 0 };
    out.rightCols(3) = Eigen::Map<MatrixXd>(matdata, 3, 3);
    return out;
  }
  virtual DblMatrix RotationJacobian(int link_ind) const {
    PRINT_AND_THROW("not implemented");
  }
  virtual bool DoesAffect(const KinBody::Link& link) {
    const vector<KinBody::LinkPtr>& links = m_body->GetLinks();
    for (int i=0; i < links.size(); ++i) {
      if (links[i].get() == &link) return true;
    }
    return false;
  }
  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() {
    return m_body->GetLinks();
  }
  virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links,
      bool only_with_geom, vector<int>& link_inds) {
    links = GetAffectedLinks();
    link_inds.resize(links.size());
    for (int i = 0; i < links.size(); ++i)
      link_inds.push_back(links[i]->GetIndex());
  }
  virtual DblVec RandomDOFValues() {
    return toDblVec(VectorXd::Random(6));
  }
  virtual vector<OpenRAVE::KinBodyPtr> GetBodies() {
    return singleton(m_body);
  }
};
typedef boost::shared_ptr<IncrementalRB> IncrementalRBPtr;

}
