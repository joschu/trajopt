#include <string>
#include "modeling.hpp"
/*
 * Algorithms for non-convex, constrained optimization
 */

namespace ipi{
namespace sco {

using std::string;
using std::vector;


enum OptStatus {
  OPT_CONVERGED,
  OPT_ITERATION_LIMIT, // hit iteration limit before convergence
  OPT_FAILED
};
static const char* OptStatus_strings[]  = {
  "CONVERGED",
  "ITERATION_LIMIT",
  "FAILED"
};
inline string statusToString(OptStatus status) {
  return OptStatus_strings[status];
}


class Optimizer {
  /*
   * Solves an optimization problem
   */
public:
  virtual OptStatus optimize() = 0;
  virtual ~Optimizer() {}
  virtual void setProblem(OptProbPtr prob) {prob_ = prob;}
  void initialize(const vector<double>& x) {x_ = x;}
  vector<double>& x() {return x_;}
protected:
  OptProbPtr prob_;
  vector<double> x_; // current solution for all variables
};

class BasicTrustRegionSQP : public Optimizer {
  /*
   * Iterates between convexifying objectives and constraints and then solving convex subproblem
   * Uses a merit function to decide whether or not to accept the step
   * merit function = objective + merit_err_coeff * | constraint_error |
   * Note: sometimes the convexified objectives and constraints lead to an infeasible subproblem
   * In that case, you should turn them into penalties and solve that problem
   * (todo: implement penalty-based sqp that gracefully handles infeasible constraints)
   */
public:
  double merit_error_coeff_,
         improve_ratio_threshold_, // minimum ratio true_improve/approx_improve to accept step
         min_trust_box_size_, // if trust region gets any smaller, exit and report convergence
         min_approx_improve_, // if model improves less than this, exit and report convergence
         min_approx_improve_frac_,
         max_iter_,
         trust_shrink_ratio_,
         trust_expand_ratio_;
  double trust_box_size_; // current size of trust region (component-wise)

  BasicTrustRegionSQP();
  BasicTrustRegionSQP(OptProbPtr prob);
  void setProblem(OptProbPtr prob);
  OptStatus optimize();
protected:
  void adjustTrustRegion(double ratio);
  void setTrustBoxConstraints(const vector<double>& x);
  void initParameters();
  ModelPtr model_;
};


}
}
