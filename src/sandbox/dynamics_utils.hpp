#include <openrave-core.h>
#include <openrave/openrave.h>
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "sco/modeling.hpp"
#include <boost/foreach.hpp>
using namespace Eigen;

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
    int n=0;
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




}
