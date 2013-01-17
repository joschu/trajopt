#include "optimizers.hpp"
#include "modeling.hpp"
#define IPI_LOG_THRESH IPI_LEVEL_DEBUG
#include "ipi/logging.hpp"
#include <boost/foreach.hpp>
#include "solver_interface.hpp"
#include "expr_ops.hpp"
#include <cmath>
#include <cstdio>
#include "sco_common.hpp"
#include "utils/stl_to_string.hpp"
using namespace std;
using namespace util;

namespace ipi {
namespace sco {

typedef vector<double> DblVec;

std::ostream& operator<<(std::ostream& o, const OptResults& r) {
  o << "Optimization results:" << endl
    << "status: " << statusToString(r.status) << endl
    << "cost values: " << Str(r.cost_vals) << endl
    << "constraint violations: " << Str(r.cnt_viols) << endl
    << "n func evals: " << r.n_func_evals << endl
    << "n qp solves: " << r.n_qp_solves << endl;
  return o;
}

//////////////////////////////////////////////////
////////// private utility functions for  sqp /////////
//////////////////////////////////////////////////



static DblVec evaluateCosts(vector<CostPtr>& costs, const DblVec& x) {
  DblVec out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) {
    out[i] = costs[i]->value(x);
  }
  return out;
}
static DblVec evaluateConstraintViols(vector<ConstraintPtr>& constraints, const DblVec& x) {
  DblVec out(constraints.size());
  for (size_t i=0; i < constraints.size(); ++i) {
    out[i] = constraints[i]->violation(x);
  }
  return out;
}
static vector<ConvexObjectivePtr> convexifyCosts(vector<CostPtr>& costs, const DblVec& x, Model* model) {
  vector<ConvexObjectivePtr> out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) {
    out[i] = costs[i]->convex(x,  model);
  }
  return out;
}
static vector<ConvexConstraintsPtr> convexifyConstraints(vector<ConstraintPtr>& cnts, const DblVec& x, Model* model) {
  vector<ConvexConstraintsPtr> out(cnts.size());
  for (size_t i=0; i < cnts.size(); ++i) {
    out[i] = cnts[i]->convex(x, model);
  }
  return out;
}

DblVec evaluateModelCosts(vector<ConvexObjectivePtr>& costs, const DblVec& x) {
  DblVec out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) {
    out[i] = costs[i]->value(x);
  }
  return out;
}
DblVec evaluateModelCntViols(vector<ConvexConstraintsPtr>& cnts, const DblVec& x) {
  DblVec out(cnts.size());
  for (size_t i=0; i < cnts.size(); ++i) {
    out[i] = cnts[i]->violation(x);
  }
  return out;
}

static vector<string> getCostNames(const vector<CostPtr>& costs) {
  vector<string> out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) out[i] = costs[i]->name();
  return out;
}
static vector<string> getCntNames(const vector<ConstraintPtr>& cnts) {
  vector<string> out(cnts.size());
  for (size_t i=0; i < cnts.size(); ++i) out[i] = cnts[i]->name();
  return out;
}

void printCostInfo(const vector<double>& old_cost_vals, const vector<double>& model_cost_vals, const vector<double>& new_cost_vals,
                  const vector<double>& old_cnt_vals, const vector<double>& model_cnt_vals, const vector<double>& new_cnt_vals,
    const vector<string>& cost_names, const vector<string>& cnt_names, double merit_coeff) {
    printf("%15s | %10s | %10s | %10s | %10s\n", "", "oldexact", "dapprox", "dexact", "ratio");
    printf("%15s | %10s---%10s---%10s---%10s\n", "COSTS", "----------", "----------", "----------", "----------");
    for (size_t i=0; i < old_cost_vals.size(); ++i) {
      double approx_improve = old_cost_vals[i] - model_cost_vals[i];
      double exact_improve = old_cost_vals[i] - new_cost_vals[i];
      string fmtstr = fabs(approx_improve) < 1e-8 ? "%15s | %10.3e | %10.3e | %10.3e | (%8.3e)\n" : "%15s | %10.3e | %10.3e | %10.3e | %10.3e\n";
      printf(fmtstr.c_str(), cost_names[i].c_str(),
                   old_cost_vals[i], approx_improve, exact_improve, exact_improve/approx_improve);
    }
    if (cnt_names.size() == 0) return;
    printf("%15s | %10s---%10s---%10s---%10s\n", "CONSTRAINTS", "----------", "----------", "----------", "----------");
    for (size_t i=0; i < old_cnt_vals.size(); ++i) {
      double approx_improve = old_cnt_vals[i] - model_cnt_vals[i];
      double exact_improve = old_cnt_vals[i] - new_cnt_vals[i];
      string fmtstr = fabs(approx_improve) < 1e-8 ? "%15s | %10.3e | %10.3e | %10.3e | (%8.3e)\n" : "%15s | %10.3e | %10.3e | %10.3e | %10.3e\n";
      printf(fmtstr.c_str(), cnt_names[i].c_str(),
                   merit_coeff*old_cnt_vals[i], merit_coeff*approx_improve, merit_coeff*exact_improve, exact_improve/approx_improve);
    }

}

// todo: use different coeffs for each constraint
vector<ConvexObjectivePtr> cntsToCosts(const vector<ConvexConstraintsPtr>& cnts, double err_coeff, Model* model) {
  vector<ConvexObjectivePtr> out;
  BOOST_FOREACH(const ConvexConstraintsPtr& cnt, cnts) {
    ConvexObjectivePtr obj(new ConvexObjective(model));
    BOOST_FOREACH(const AffExpr& aff, cnt->eqs_) {
      obj->addAbs(aff, err_coeff);
    }
    BOOST_FOREACH(const AffExpr& aff, cnt->ineqs_) {
      obj->addAbs(aff, err_coeff);
    }
    out.push_back(obj);
  }
  return out;
}

void Optimizer::addCallback(const Callback& cb) {
  callbacks_.push_back(cb);
}
void Optimizer::callCallbacks(const DblVec& x) {
  for (int i=0; i < callbacks_.size(); ++i) {
    callbacks_[i](x);
  }
}

BasicTrustRegionSQP::BasicTrustRegionSQP() {
  initParameters();
}
BasicTrustRegionSQP::BasicTrustRegionSQP(OptProbPtr prob) {
  initParameters();
  setProblem(prob);
}

void BasicTrustRegionSQP::initParameters() {
  merit_error_coeff_ = 100;
  improve_ratio_threshold_ = .25;
  min_trust_box_size_ = 1e-4;
  min_approx_improve_= 1e-4;
  min_approx_improve_frac_ = -INFINITY;
  max_iter_ = 50;
  trust_shrink_ratio_=.1;
  trust_expand_ratio_ = 1.5;
  trust_box_size_ = 1e-1;
}

void BasicTrustRegionSQP::setProblem(OptProbPtr prob) {
  Optimizer::setProblem(prob);
  model_ = prob->getModel();
}

void BasicTrustRegionSQP::adjustTrustRegion(double ratio) {
  trust_box_size_ *= ratio;
}
void BasicTrustRegionSQP::setTrustBoxConstraints(const DblVec& x) {
  vector<Var>& vars = prob_->getVars();
  assert(vars.size() == x.size());
  DblVec& lb=prob_->getLowerBounds(), ub=prob_->getUpperBounds();
  for (size_t i=0; i < x.size(); ++i) {
    model_->setVarBounds(vars[i], fmax(x[i] - trust_box_size_, lb[i]),
                                  fmin(x[i] + trust_box_size_, ub[i]));
  }
}

OptStatus BasicTrustRegionSQP::optimize() {

  vector<string> cost_names = getCostNames(prob_->getCosts());
  vector<ConstraintPtr> constraints = prob_->getConstraints();
  vector<string> cnt_names = getCntNames(constraints);

  DblVec& x_ = results_.x; // just so I don't have to rewrite code
  x_ = prob_->getClosestFeasiblePoint(x_);
  
  assert(x_.size() == prob_->getVars().size());
  assert(prob_->getCosts().size() > 0 || constraints.size() > 0);

  OptStatus retval = INVALID;


  for (size_t iter = 1; ; ++iter) {
    callCallbacks(x_);


    IPI_LOG_DEBUG("current iterate: %s", Str(x_));
    IPI_LOG_INFO("iteration %i", iter);

    // speed optimization: if you just evaluated the cost when doing the line search, use that
    if (results_.cost_vals.empty()) { //only happens on the first iteration
      results_.cnt_viols = evaluateConstraintViols(constraints,  x_);
      results_.cost_vals = evaluateCosts(prob_->getCosts(), x_);
      assert(results_.n_func_evals == 0);
      ++results_.n_func_evals;
    }


    vector<ConvexObjectivePtr> cost_models = convexifyCosts(prob_->getCosts(), x_, model_.get());
    vector<ConvexConstraintsPtr> cnt_models = convexifyConstraints(constraints, x_, model_.get());
    model_->update();
    BOOST_FOREACH(ConvexObjectivePtr& cost, cost_models) cost->addConstraintsToModel();
    BOOST_FOREACH(ConvexConstraintsPtr& cnt, cnt_models) cnt->addConstraintsToModel();
    model_->update();
    QuadExpr objective;
    BOOST_FOREACH(ConvexObjectivePtr& co, cost_models) exprInc(objective, co->quad_);
//    objective = cleanupExpr(objective);
    model_->setObjective(objective);

    if (logging::filter() >= IPI_LEVEL_DEBUG) {
//      DblVec model_cost_vals;
//      BOOST_FOREACH(ConvexObjectivePtr& cost, cost_models) {
//        model_cost_vals.push_back(cost->value(x));
//      }
//      IPI_LOG_DEBUG("model costs %s should equalcosts  %s", printer(model_cost_vals), printer(cost_vals));
    }

    bool solving_slack_problem = false;
    vector<ConvexObjectivePtr> cnt_cost_models;
    while (trust_box_size_ >= min_trust_box_size_) {

      setTrustBoxConstraints(x_);

      CvxOptStatus status = model_->optimize();
      ++results_.n_qp_solves;

      if (status != CVX_SOLVED) {
        if (solving_slack_problem) {
          model_->writeToFile("/tmp/fail.lp");
          IPI_ABORT("inequalities turned into penalties but problem is still infeasible. written to /tmp/fail.lp");
        }
        IPI_LOG_WARNING("problem was infeasible. optimizing relaxed problem");
//        model_->writeToFile("/tmp/infeas.lp");
//        IPI_LOG_WARNING("infeasible model written to /tmp/infeas.lp");

        BOOST_FOREACH(ConvexConstraintsPtr& cnt, cnt_models) cnt->removeFromModel();
        cnt_cost_models = cntsToCosts(cnt_models, merit_error_coeff_, model_.get());
        model_->update();
        BOOST_FOREACH(ConvexObjectivePtr& cost, cnt_cost_models) cost->addConstraintsToModel();
        model_->update();
        QuadExpr cnt_objective = objective; // cnt_objective = orig objective + constraint penalties
        BOOST_FOREACH(ConvexObjectivePtr& co, cnt_cost_models) exprInc(cnt_objective, co->quad_);
        model_->setObjective(cnt_objective);
        model_->update();
//        IPI_LOG_INFO("relaxed model written to /tmp/relaxed.lp");
//        model_->writeToFile("/tmp/relaxed.lp");
        solving_slack_problem = true;
        continue;
      }

      else if (status == CVX_FAILED) {
        IPI_LOG_ERR("convex solver failed! aborting. saving model to /tmp/fail.lp");
        model_->writeToFile("/tmp/fail.lp");
        retval = OPT_FAILED;
        goto cleanup;
      }
      DblVec model_var_vals = model_->getVarValues(model_->getVars());

      DblVec model_cost_vals = evaluateModelCosts(cost_models, model_var_vals);
      DblVec model_cnt_viols = evaluateModelCntViols(cnt_models, model_var_vals);

      // the n variables of the OptProb happen to be the first n variables in the Model
      DblVec new_x(model_var_vals.begin(), model_var_vals.begin() + x_.size());

      if (solving_slack_problem && logging::filter() >= IPI_LEVEL_DEBUG) {
        DblVec model_cnt_viols2 = evaluateModelCosts(cnt_cost_models, model_var_vals);
        IPI_LOG_DEBUG("SHOULD BE THE SAME: %.2f*%s ?= %s", merit_error_coeff_, Str(model_cnt_viols), Str(model_cnt_viols2));
      }

      DblVec new_cost_vals = evaluateCosts(prob_->getCosts(), new_x);
      DblVec new_cnt_viols = evaluateConstraintViols(constraints, new_x);
      ++results_.n_func_evals;

      double old_merit = vecSum(results_.cost_vals) + merit_error_coeff_ * vecSum(results_.cnt_viols);
      double model_merit = vecSum(model_cost_vals) + merit_error_coeff_ * vecSum(model_cnt_viols);
      double new_merit = vecSum(new_cost_vals) + merit_error_coeff_ * vecSum(new_cnt_viols);
      double approx_merit_improve = old_merit - model_merit;
      double exact_merit_improve = old_merit - new_merit;
      double merit_improve_ratio = exact_merit_improve / approx_merit_improve;

      if (logging::filter() >= IPI_LEVEL_INFO) {
        IPI_LOG_INFO("");
        printCostInfo(results_.cost_vals, model_cost_vals, new_cost_vals, results_.cnt_viols, model_cnt_viols, new_cnt_viols, cost_names, cnt_names, merit_error_coeff_);
        printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e\n", "TOTAL", old_merit, approx_merit_improve, exact_merit_improve, merit_improve_ratio);
      }


      if (approx_merit_improve < -1e-6) {
        IPI_LOG_ERR("approximate merit function got worse. (convexification is probably wrong to zeroth order)");
      }
      if (approx_merit_improve < min_approx_improve_) {
        IPI_LOG_INFO("converged because improvement was small (%.3e < %.3e)", approx_merit_improve, min_approx_improve_);
        retval = OPT_CONVERGED;
        goto cleanup;
      }
      if (approx_merit_improve/old_merit < min_approx_improve_frac_) {
        IPI_LOG_INFO("converged because improvement ratio was small (%.3e < %.3e)", approx_merit_improve/old_merit, min_approx_improve_frac_);
        retval = OPT_CONVERGED;
        goto cleanup;
      }
      else if (exact_merit_improve < 0 || merit_improve_ratio < improve_ratio_threshold_) {
        adjustTrustRegion(trust_shrink_ratio_);
        IPI_LOG_INFO("shrunk trust region. new box size: %.4f", trust_box_size_);
      }
      else {
        x_ = new_x;
        results_.cost_vals = new_cost_vals;
        results_.cnt_viols = new_cnt_viols;
        adjustTrustRegion(trust_expand_ratio_);
        IPI_LOG_INFO("expanded trust region. new box size: %.4f", trust_box_size_);
        break;
      }
    }

    if (trust_box_size_ < min_trust_box_size_) {
      IPI_LOG_INFO("converged because trust region is tiny");
      retval = OPT_CONVERGED;
      goto cleanup;
    }
    else if (iter >= max_iter_) {
      IPI_LOG_INFO("iteration limit");
      retval = OPT_ITERATION_LIMIT;
      goto cleanup;
    }
  }


  assert(0 && "unreachable");

  cleanup:
  assert(retval != INVALID && "should never happen");
  results_.status = retval;
  IPI_LOG_INFO("\n==================\n%s==================", Str(results_));

  /**
   * We don't need cleanup because the ConvexObjective / ConvexConstraint objects
   * get removed when they go out of scope. So we don't really need a goto here
   */


  return retval;

}


}}
