#pragma once
#include "trajopt/common.hpp"
namespace trajopt {

struct TrajPlotter {
  OpenRAVE::EnvironmentBasePtr m_env;
  ConfigurationPtr m_config;
  VarArray m_trajvars;
  vector<PlotterPtr> m_plotters;
  std::set<KinBody::LinkPtr> m_links; // links for which we'll plot the trajectory

  TrajPlotter(OR::EnvironmentBasePtr env, ConfigurationPtr config, const VarArray& trajvars);
  void Add(const vector<CostPtr>& costs);
  void Add(const vector<ConstraintPtr>& constraints);
  void Add(const vector<PlotterPtr>& plotters);
  void Add(PlotterPtr plotter);
  void AddLink(OpenRAVE::KinBody::LinkPtr link);
  void OptimizerCallback(OptProb*, DblVec& x);

};
typedef boost::shared_ptr<TrajPlotter> TrajPlotterPtr;

}
