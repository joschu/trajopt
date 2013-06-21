#include "trajopt/configuration_space.hpp"

namespace trajopt {

struct StaticObject: public Configuration {
  OpenRAVE::KinBodyPtr m_body;

  StaticObject(OpenRAVE::KinBodyPtr body) :
      m_body(body) {}
  virtual void SetDOFValues(const DblVec& dofs) {}
  virtual DblVec GetDOFValues() {return DblVec();};
  virtual int GetDOF() const {return 0;}
  virtual void GetDOFLimits(DblVec&, DblVec&) const {}
  virtual OpenRAVE::EnvironmentBasePtr GetEnv() {
    return m_body->GetEnv();
  }
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const {
    return MatrixXd::Zero(3,0);
  }
  virtual DblMatrix RotationJacobian(int link_ind) const {
    PRINT_AND_THROW("not implemented");
  }
  virtual bool DoesAffect(const KinBody::Link& link) {
    BOOST_FOREACH(const KinBody::LinkPtr& link1, m_body->GetLinks()) {
      if (link1.get() == &link)
        return true;
    }
    return false;
  }
  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() {
    return m_body->GetLinks();
  }
  virtual void GetAffectedLinks(std::vector<OpenRAVE::KinBody::LinkPtr>& links,
      bool only_with_geom, vector<int>& link_inds) {
    links = GetAffectedLinks();
    link_inds.resize(links.size());
    for (int i = 0; i < links.size(); ++i)
      link_inds.push_back(links[i]->GetIndex());
  }
  virtual DblVec RandomDOFValues() {
    return toDblVec(VectorXd::Random(0));
  }
  virtual vector<OpenRAVE::KinBodyPtr> GetBodies() {return vector<OpenRAVE::KinBodyPtr>(1, m_body);}
};
typedef boost::shared_ptr<StaticObject> StaticObjectPtr;

}