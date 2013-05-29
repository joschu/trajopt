#include <openrave-core.h>
#include <openrave/openrave.h>
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "sco/modeling.hpp"
#include <boost/foreach.hpp>
#include "sco/expr_op_overloads.hpp"
using namespace Eigen;
using namespace std;

namespace trajopt {

typedef BasicArray<AffExpr> AffArray;
typedef BasicArray<Cnt> CntArray;

MatrixXd getTraj(const DblVec& x, const AffArray& arr) {
  MatrixXd out(arr.rows(), arr.cols());
  for (int i=0; i < arr.rows(); ++i) {
    for (int j=0; j < arr.cols(); ++j) {
      out(i,j) = arr(i,j).value(x);
    }
  }
  return out;
}

void AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<string>& name_prefix, const vector<VarArray*>& newvars) {
  int n_arr = name_prefix.size();
  assert(n_arr == newvars.size());

  vector<MatrixXi> index(n_arr);
  for (int i=0; i < n_arr; ++i) {
    newvars[i]->resize(rows, cols[i]);
    index[i].resize(rows, cols[i]);
  }

  vector<string> names;
  int var_idx = prob.getNumVars();
  for (int i=0; i < rows; ++i) {
    for (int k=0; k < n_arr; ++k) {
      for (int j=0; j < cols[k]; ++j) {
        index[k](i,j) = var_idx;
        names.push_back( (boost::format("%s_%i_%i")%name_prefix[k]%i%j).str() );
        ++var_idx;
      }
    }
  }
  prob.createVariables(names); // note that w,r, are both unbounded

  const vector<Var>& vars = prob.getVars();
  for (int k=0; k < n_arr; ++k) {
    for (int i=0; i < rows; ++i) {
      for (int j=0; j < cols[k]; ++j) {
        (*newvars[k])(i,j) = vars[index[k](i,j)];
      }
    }
  }
}

void AddVarArray(OptProb& prob, int rows, int cols, const string& name_prefix, VarArray& newvars) {
  vector<VarArray*> arrs(1, &newvars);
  vector<string> prefixes(1, name_prefix);
  vector<int> colss(1, cols);
  AddVarArrays(prob, rows, colss, prefixes, arrs);
}

template <typename T> // var or expr
void FiniteDifferences(const BasicArray<T>& x, AffArray& dx, double dt) {
  dx.resize(x.rows()-1, x.cols());
  for (int i=0; i < x.rows(); ++i) {
    for (int j=0; j < x.cols(); ++j) {
      dx(i,j) = x(i+1,j) - x(i,j);
    }
  }
}


class CompositeConfig : public Configuration {
public:
  
  vector<ConfigurationPtr> m_configs;
  vector<int> m_startInds;
  
  CompositeConfig(ConfigurationPtr config0, ConfigurationPtr config1) {
    vector<ConfigurationPtr> configs;
    configs.push_back(config0);
    configs.push_back(config1);
    Init(configs);
  }
  CompositeConfig(const vector<ConfigurationPtr>& configs) {
    Init(configs);
  }
  void Init(const vector<ConfigurationPtr>& configs)  {
    m_configs = configs;
    int startInd = 0;
    BOOST_FOREACH(const ConfigurationPtr& config, m_configs) {
      m_startInds.push_back(startInd);
      startInd += config->GetDOF();
    }
  }
  virtual void SetDOFValues(const DblVec& dofs) {
    int n=0;
    BOOST_FOREACH(const ConfigurationPtr& config, m_configs) {
      int d = config->GetDOF();
      DblVec part_dofs(dofs.begin()+n, dofs.begin()+n+d);
      config->SetDOFValues(part_dofs);      
      n += d;
    }
  }
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const {
    lower.clear();
    upper.clear();
    BOOST_FOREACH(const ConfigurationPtr& config, m_configs) {
      DblVec part_lower, part_upper;
      config->GetDOFLimits(part_lower, part_upper);
      lower.insert(lower.end(), part_lower.begin(), part_lower.end());
      upper.insert(upper.end(), part_upper.begin(), part_upper.end());
    }
    
  }
  virtual DblVec GetDOFValues() {
    DblVec out;
    BOOST_FOREACH(const ConfigurationPtr& config, m_configs) {
      DblVec part_vals = config->GetDOFValues();
      out.insert(out.end(), part_vals.begin(), part_vals.end());
    }        
    return out;
  }
  virtual int GetDOF() const {
    int out=0;
    BOOST_FOREACH(const ConfigurationPtr& config, m_configs) {
      out += config->GetDOF();
    }    
    return out;    
  }
  virtual OpenRAVE::EnvironmentBasePtr GetEnv() {return m_configs[0]->GetEnv();}
  
  
  static const int PART_FACTOR = 1024;
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const {
    int i_part = link_ind / PART_FACTOR;
    DblMatrix out = MatrixXd::Zero(3, GetDOF());
    int i_startDof = m_startInds[i_part];
    out.block(0, i_startDof, 3, m_configs[i_part]->GetDOF()) = m_configs[i_part]->PositionJacobian(link_ind%PART_FACTOR, pt);
    return out;
  }
  virtual DblMatrix RotationJacobian(int link_ind) const {PRINT_AND_THROW("not implemented");}
  virtual bool DoesAffect(const KinBody::Link& link) {
    bool out = false;
    BOOST_FOREACH(ConfigurationPtr& config, m_configs) {
      out |= config->DoesAffect(link);
    }
    return out;
  };
  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() {
    std::vector<KinBody::LinkPtr> out;
    BOOST_FOREACH(ConfigurationPtr& config, m_configs) {
      std::vector<KinBody::LinkPtr> part_links = config->GetAffectedLinks();
      out.insert(out.end(), part_links.begin(), part_links.end());
    }
    return out;
    
  }
  virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) {
    links.clear();
    link_inds.clear();
    int i_part = 0;
    BOOST_FOREACH(ConfigurationPtr& config, m_configs) {
      std::vector<KinBody::LinkPtr> part_links;
      vector<int> part_link_inds;
      config->GetAffectedLinks(part_links, only_with_geom, part_link_inds);
      BOOST_FOREACH(int& i, part_link_inds) i += PART_FACTOR*i_part;
      links.insert(links.end(), part_links.begin(), part_links.end());
      link_inds.insert(link_inds.end(), part_link_inds.begin(), part_link_inds.end());
      ++i_part;
    }
  }
  virtual vector<OpenRAVE::KinBodyPtr> GetBodies() {
    vector<OpenRAVE::KinBodyPtr> out;
    for (int i=0; i < m_configs.size(); ++i) {
      vector<OpenRAVE::KinBodyPtr> newvec;
      out.insert(out.end(), newvec.begin(), newvec.end());
    }
    return out;
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



OpenRAVE::Vector toRaveQuat(const Vector4d& v) {
  return OpenRAVE::Vector(v[0], v[1], v[2], v[3]);
}
OpenRAVE::Vector toRave(const Vector3d& v) {
  return OpenRAVE::Vector(v[0], v[1], v[2]);
}


template<typename T>
void extend(vector<T>& a, const vector<T>& b) {
  a.insert(a.end(), b.begin(), b.end());
}

template<typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b, const vector<T>& c) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  out.insert(out.end(), c.begin(), c.end());
  return out;
}
template<typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b, const vector<T>& c, const vector<T>& d) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  out.insert(out.end(), c.begin(), c.end());
  out.insert(out.end(), d.begin(), d.end());
  return out;
}
template<typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b, const vector<T>& c, const vector<T>& d, const vector<T>& e) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  out.insert(out.end(), c.begin(), c.end());
  out.insert(out.end(), d.begin(), d.end());
  out.insert(out.end(), e.begin(), e.end());
  return out;
}
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
    BOOST_FOREACH(const KinBody::LinkPtr& link1, m_body->GetLinks()) {
      if (link1.get() == &link)
        return true;
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
    return vector<OpenRAVE::KinBodyPtr>(1, m_body);
  }
};
typedef boost::shared_ptr<IncrementalRB> IncrementalRBPtr;

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
};
typedef boost::shared_ptr<StaticObject> StaticObjectPtr;

template<typename S, typename T, typename U>
vector<U> cross1(const vector<S>& a, const vector<T>& b) {
  vector<U> c(3);
  c[0] = a[1] * b[2] - a[2] * b[1];
  c[1] = a[2] * b[0] - a[0] * b[2];
  c[2] = a[0] * b[1] - a[1] * b[0];
  return c;
}

void exprInc(AffExprVector& a, const AffExprVector&  b) {
  for (int i=0; i < a.size(); ++i) {
    exprInc(a[i], b[i]);
  }
}
void exprDec(AffExprVector& a, const AffExprVector&  b) {
  for (int i=0; i < a.size(); ++i) {
    exprDec(a[i], b[i]);
  }
}
AffExprVector linearizedCrossProduct(const DblVec& x, const AffExprVector& a, const AffExprVector& b) {
  assert(a.size() == 3 && b.size() == 3);
  DblVec aval(3), bval(3);
  for (int i=0; i < 3; ++i) {
    aval[i] = a[i].value(x);
    bval[i] = b[i].value(x);
  }
  AffExprVector c(3);
  exprInc(c, cross1<AffExpr, double, AffExpr>(a, bval));
  exprInc(c, cross1<double, AffExpr, AffExpr>(aval, b));
  DblVec acrossbval = cross1<double,double,double>(aval, bval);
  for (int i=0; i<3; ++i) exprDec(c[i], acrossbval[i]);
  return c;
}



AffExprVector transformExpr(const OR::Transform& T, const AffExprVector& v) {
  OR::TransformMatrix M(T);
  AffExprVector out(3);
  for (int i=0; i < 3; ++i) {
    for (int j=0; j < 3; ++j) {
      exprInc(out[i], M.rot(i,j) * v[j]);
    }
    exprInc(out[i], T.trans[i]);
  }
  return out;
}
AffExprVector rotateExpr(const OR::Vector& rot, const AffExprVector& v) {
  OR::Transform T;
  T.rot = rot;
  return transformExpr(T, v);
}


}
