#include "solver_interface.hpp"
#include "gurobi_interface.hpp"
#include "utils/logging.hpp"
extern "C" {
#include "gurobi_c.h"
}
#include <boost/foreach.hpp>
#include "sco_common.hpp"
#include <map>
#include <utility>
#include "macros.h"
#include <sstream>
#include <stdexcept>
#include <iostream>
#include "utils/stl_to_string.hpp"

using namespace std;

namespace sco {

GRBenv* gEnv;

void simplify2(vector<int>& inds, vector<double>& vals) {
  typedef std::map<int, double> Int2Double;
  Int2Double ind2val;
  for (unsigned i=0; i < inds.size(); ++i) {
    if (vals[i] != 0) ind2val[inds[i]] += vals[i];
  }
  inds.resize(ind2val.size());
  vals.resize(ind2val.size());
  int i_new = 0;
  BOOST_FOREACH(Int2Double::value_type& iv, ind2val) {
    inds[i_new] = iv.first;
    vals[i_new] = iv.second;
    ++i_new;
  }
}
#if 0
void simplify(vector<int>& inds, vector<double>& vals) {
  // first find the largest element of inds
  // make an array of that size
  //

  assert(inds.size() == vals.size());
  int min_ind = *std::min_element(inds.begin(), inds.end());
  int max_ind = *std::max_element(inds.begin(), inds.end());
  int orig_size = inds.size();
  int work_size = max_ind - min_ind + 1;
  vector<double> work_vals(work_size, 0);

  for (int i=0; i < orig_size; ++i) {
    work_vals[inds[i] - min_ind] += vals[i];
  }


  int i_new = 0;
  for (int i=0; i < work_size; ++i) {
    if (work_vals[i] != 0) {
      vals[i_new] = work_vals[i];
      inds[i_new] = i + min_ind;
      ++i_new;
    }
  }

  inds.resize(i_new);
  vals.resize(i_new);

}
#endif

#define ENSURE_SUCCESS(expr) do {\
    bool error = expr;\
    if (error) {\
      printf("GRB error: %s while evaluating %s at %s:%i\n", GRBgeterrormsg(gEnv), #expr, __FILE__,__LINE__);\
      abort();\
    }\
} while(0)

ModelPtr createGurobiModel() {
  ModelPtr out(new GurobiModel());
  return out;
}

GurobiModel::GurobiModel() {
  if (!gEnv) {
    GRBloadenv(&gEnv, NULL);
    if (util::GetLogLevel() < util::LevelDebug) {
      ENSURE_SUCCESS(GRBsetintparam(gEnv, "OutputFlag",0));
    }
  }
  GRBnewmodel(gEnv, &model,"problem",0, NULL, NULL, NULL,NULL, NULL);
}

static vector<int> vars2inds(const vector<Var>& vars) {
  vector<int> inds(vars.size());
  for (size_t i=0; i < inds.size(); ++i) inds[i] = vars[i].var_rep->index;
  return inds;
}

Var GurobiModel::addVar(const string& name) {
  ENSURE_SUCCESS(GRBaddvar(model, 0, NULL, NULL, 0, -GRB_INFINITY, GRB_INFINITY, GRB_CONTINUOUS, const_cast<char*>(name.c_str())));
  vars.push_back(new VarRep(vars.size(), name, this));
  return vars.back();
}

Var GurobiModel::addVar(const string& name, double lb, double ub) {
  ENSURE_SUCCESS(GRBaddvar(model, 0, NULL, NULL, 0, lb, ub, GRB_CONTINUOUS, const_cast<char*>(name.c_str())));
  vars.push_back(new VarRep(vars.size(), name, this));
  return vars.back();
}


Cnt GurobiModel::addEqCnt(const AffExpr& expr, const string& name) {
  LOG_DEBUG("adding ineq: %s <= 0", CSTR(expr));
  vector<int> inds = vars2inds(expr.vars);
  vector<double> vals = expr.coeffs;
  simplify2(inds, vals);
  ENSURE_SUCCESS(GRBaddconstr(model, inds.size(),
       const_cast<int*>(inds.data()), const_cast<double*>(vals.data()), GRB_EQUAL, -expr.constant, const_cast<char*>(name.c_str())));
  cnts.push_back(new CntRep(cnts.size(), this));
  return cnts.back();
}
Cnt GurobiModel::addIneqCnt(const AffExpr& expr, const string& name) {
  LOG_DEBUG("adding ineq: %s <= 0", CSTR(expr));
  vector<int> inds = vars2inds(expr.vars);
  vector<double> vals = expr.coeffs;
  simplify2(inds, vals);
  ENSURE_SUCCESS(GRBaddconstr(model, inds.size(),
      const_cast<int*>(inds.data()), const_cast<double*>(vals.data()), GRB_LESS_EQUAL, -expr.constant, const_cast<char*>(name.c_str())));
  cnts.push_back(new CntRep(cnts.size(), this));
  return cnts.back();
}
Cnt GurobiModel::addIneqCnt(const QuadExpr&, const string& name) {
  PRINT_AND_THROW("NOT IMPLEMENTED");
  return 0;
}

void resetIndices(VarVector& vars) {
  for (size_t i=0; i < vars.size(); ++i) vars[i].var_rep[i].index = i;
}
void resetIndices(vector<Cnt>& cnts) {
  for (size_t i=0; i < cnts.size(); ++i) cnts[i].cnt_rep[i].index = i;
}

void GurobiModel::removeVar(const Var& var) {
  assert(var.var_rep->creator == this);
  ENSURE_SUCCESS(GRBdelvars(model, 1, &var.var_rep->index));
  var.var_rep->removed = true;
}

void GurobiModel::removeCnt(const Cnt& cnt) {
  assert(cnt.cnt_rep->creator == this);
  ENSURE_SUCCESS(GRBdelconstrs(model, 1, &cnt.cnt_rep->index));
  cnt.cnt_rep->removed = true;
}


void GurobiModel::setVarBounds(const Var& var, double lower, double upper) {
  assert(var.var_rep->creator == this);
  ENSURE_SUCCESS(GRBsetdblattrelement(model, GRB_DBL_ATTR_LB, var.var_rep->index, lower));
  ENSURE_SUCCESS(GRBsetdblattrelement(model, GRB_DBL_ATTR_UB, var.var_rep->index, upper));
}

void GurobiModel::setVarBounds(const VarVector& vars, const vector<double>& lower, const vector<double>& upper) {
  assert(vars.size() == lower.size() && vars.size() == upper.size());
  ENSURE_SUCCESS(GRBsetdblattrarray(model, GRB_DBL_ATTR_LB, 0, vars.size(), const_cast<double*>(lower.data())));
  ENSURE_SUCCESS(GRBsetdblattrarray(model, GRB_DBL_ATTR_UB, 0, vars.size(), const_cast<double*>(upper.data())));
}


double GurobiModel::getVarValue(const Var& var) const {
  assert(var.var_rep->creator == this);
  double out;
  ENSURE_SUCCESS(GRBgetdblattrelement(model, GRB_DBL_ATTR_X, var.var_rep->index, &out));
  return out;
}
vector<double> GurobiModel::getVarValues(const vector<Var>& vars) const {
  assert((vars.size() == 0) || (vars[0].var_rep->creator == this));
  vector<int> inds = vars2inds(vars);
  vector<double> out(inds.size());
  ENSURE_SUCCESS(GRBgetdblattrlist(model, GRB_DBL_ATTR_X, inds.size(), inds.data(), out.data()));
  return out;
}

CvxOptStatus GurobiModel::optimize(){
  ENSURE_SUCCESS(GRBoptimize(model));
  int status;
  GRBgetintattr(model, GRB_INT_ATTR_STATUS, &status);
  if (status == GRB_OPTIMAL) {
    double objval; GRBgetdblattr(model, GRB_DBL_ATTR_OBJVAL, &objval);
    LOG_DEBUG("solver objective value: %.3e", objval);
    return CVX_SOLVED;
  }
  else if (status == GRB_INFEASIBLE) return CVX_INFEASIBLE;
  else return CVX_FAILED;
}
CvxOptStatus GurobiModel::optimizeFeasRelax(){
  double lbpen=GRB_INFINITY, ubpen = GRB_INFINITY,  rhspen=1;
  ENSURE_SUCCESS(GRBfeasrelax(model, 0/*sum of viol*/, 0/*just minimize cost of viol*/, &lbpen, &ubpen, &rhspen, NULL));
  return optimize();
}

void GurobiModel::setObjective(const AffExpr& expr) {
  GRBdelq(model);

  int nvars;
  GRBgetintattr(model, GRB_INT_ATTR_NUMVARS, &nvars);
  assert(nvars == (int)vars.size());

  vector<double> obj(nvars, 0);
  for (size_t i=0; i < expr.size(); ++i) {
    obj[expr.vars[i].var_rep->index] += expr.coeffs[i];
  }
  ENSURE_SUCCESS(GRBsetdblattrarray(model, "Obj", 0, nvars, obj.data()));
  GRBsetdblattr(model, "ObjCon", expr.constant);
}

void GurobiModel::setObjective(const QuadExpr& quad_expr) {
  setObjective(quad_expr.affexpr);
  vector<int> inds1 = vars2inds(quad_expr.vars1);
  vector<int> inds2 = vars2inds(quad_expr.vars2);
  GRBaddqpterms(model, quad_expr.coeffs.size(), const_cast<int*>(inds1.data()),
      const_cast<int*>(inds2.data()), const_cast<double*>(quad_expr.coeffs.data()));
}

void GurobiModel::writeToFile(const string& fname) {
  ENSURE_SUCCESS(GRBwrite(model, fname.c_str()));
}


void GurobiModel::update() {
  ENSURE_SUCCESS(GRBupdatemodel(model));

  {
  int inew = 0;
  BOOST_FOREACH(const Var& var, vars) {
    if (!var.var_rep->removed) {
      vars[inew] = var;
      var.var_rep->index = inew;
      ++inew;
    }
    else delete var.var_rep;
  }
  vars.resize(inew);
  }
  {
  int inew = 0;
  BOOST_FOREACH(const Cnt& cnt, cnts) {
    if (!cnt.cnt_rep->removed) {
      cnts[inew] = cnt;
      cnt.cnt_rep->index = inew;
      ++inew;
    }
    else delete cnt.cnt_rep;
  }
  cnts.resize(inew);
  }
}

VarVector GurobiModel::getVars() const {
  return vars;
}

GurobiModel::~GurobiModel() {
  ENSURE_SUCCESS(GRBfreemodel(model));
}

}

