#pragma once
#include <openrave/openrave.h>
#include "trajopt/common.hpp"
#include "incremental_rb.hpp"


namespace trajopt {

struct O3Helper {
  /**
  Class with a bunch of methods that help you optimize over orientations
  */
  OpenRAVE::KinBodyPtr m_body;
  vector<IncrementalRBPtr> m_rbs;
  VarArray m_orivars;
  O3Helper(const OpenRAVE::KinBodyPtr body, const VarArray& vars);  
  void OptimizerCallback(OptProb*, DblVec& x);
  void ConfigureOptimizer(Optimizer&);
  void AddAngVelCosts(OptProb&, double coeff);
};



struct AngVelCost: public Cost {
  vector<IncrementalRBPtr> m_rbs;
  VarArray m_r;
  double m_coeff;
public:
  AngVelCost(vector<IncrementalRBPtr> rbs, const VarArray& r, double coeff);
  double value(const DblVec& x);
  ConvexObjectivePtr convex(const DblVec& x, Model* model);
};


MatrixXd getW(const MatrixXd& qs, double dt);

}
