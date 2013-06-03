#pragma once
#include "trajopt/configuration_space.hpp"

namespace trajopt {
  
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


}