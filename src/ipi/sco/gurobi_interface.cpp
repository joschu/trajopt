#ifdef HAVE_GUROBI
#include "solver_interface.hpp"
#include "gurobi_interface.hpp"
#define IPI_LOG_THRESH IPI_LEVEL_INFO
#include "ipi/logging.hpp"
extern "C" {
#include "gurobi_c.h"
}
#include <boost/foreach.hpp>
#include "sco_common.hpp"
#include <map>
#include <utility>
using namespace std;

namespace ipi {
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


#define abortIfError(error)\
  if (error) {\
    IPI_ABORT("GRB error: %s", GRBgeterrormsg(gEnv));\
  }\

ModelPtr createGurobiInterface() {
  ModelPtr out(new GurobiInterface());
  return out;
}

GurobiInterface::GurobiInterface() {
  if (!gEnv) {
    GRBloadenv(&gEnv, NULL);
    if (logging::filter() < IPI_LEVEL_DEBUG) {
      int error = GRBsetintparam(gEnv, "OutputFlag",0);
      abortIfError(error);
    }
  }
  GRBnewmodel(gEnv, &model,"problem",0, NULL, NULL, NULL,NULL, NULL);
}

static vector<int> vars2inds(const vector<Var>& vars) {
  vector<int> inds(vars.size());
  for (size_t i=0; i < inds.size(); ++i) inds[i] = vars[i].var_rep->index;
  return inds;
}

Var GurobiInterface::addVar(const string& name) {
  int error = GRBaddvar(model, 0, NULL, NULL, 0, -GRB_INFINITY, GRB_INFINITY, GRB_CONTINUOUS, const_cast<char*>(name.c_str()));
  abortIfError(error);
  vars.push_back(new VarRep(vars.size(), this));
  return vars.back();
}

Var GurobiInterface::addVar(const string& name, double lb, double ub) {
  int error = GRBaddvar(model, 0, NULL, NULL, 0, lb, ub, GRB_CONTINUOUS, const_cast<char*>(name.c_str()));
  abortIfError(error);
  vars.push_back(new VarRep(vars.size(), this));
  return vars.back();
}


Cnt GurobiInterface::addEqCnt(const AffExpr& expr, const string& name) {
  IPI_LOG_DEBUG("adding ineq: %s <= 0", expr);
  vector<int> inds = vars2inds(expr.vars);
  vector<double> vals = expr.coeffs;
  simplify2(inds, vals);
  int error = GRBaddconstr(model, inds.size(),
       const_cast<int*>(inds.data()), const_cast<double*>(vals.data()), GRB_EQUAL, -expr.constant, const_cast<char*>(name.c_str()));
  abortIfError(error);
  cnts.push_back(new CntRep(cnts.size(), this));
  return cnts.back();
}
Cnt GurobiInterface::addIneqCnt(const AffExpr& expr, const string& name) {
  IPI_LOG_DEBUG("adding ineq: %s <= 0", expr);
  vector<int> inds = vars2inds(expr.vars);
  vector<double> vals = expr.coeffs;
//  cout << "original inds" << printer(inds) << endl;
//  cout << "original vals" << printer(vals) << endl;
  simplify2(inds, vals);
//  cout << "new inds" << printer(inds) << endl;
//  cout << "new vals" << printer(vals) << endl;
  int error = GRBaddconstr(model, inds.size(),
      const_cast<int*>(inds.data()), const_cast<double*>(vals.data()), GRB_LESS_EQUAL, -expr.constant, const_cast<char*>(name.c_str()));
  abortIfError(error);
  cnts.push_back(new CntRep(cnts.size(), this));
  return cnts.back();
}
Cnt GurobiInterface::addIneqCnt(const QuadExpr&, const string& name) {
  IPI_ABORT("NOT IMPLEMENTED");
  return 0;
}

void resetIndices(VarVector& vars) {
  for (size_t i=0; i < vars.size(); ++i) vars[i].var_rep[i].index = i;
}
void resetIndices(vector<Cnt>& cnts) {
  for (size_t i=0; i < cnts.size(); ++i) cnts[i].cnt_rep[i].index = i;
}

void GurobiInterface::removeVar(const Var& var) {
  int error = GRBdelvars(model, 1, &var.var_rep->index);
  abortIfError(error);
  var.var_rep->removed = true;
}

void GurobiInterface::removeCnt(const Cnt& cnt) {
  int error = GRBdelconstrs(model, 1, &cnt.cnt_rep->index);
  abortIfError(error);
  cnt.cnt_rep->removed = true;
}


void GurobiInterface::setVarBounds(const Var& var, double lower, double upper) {
  int error = GRBsetdblattrelement(model, GRB_DBL_ATTR_LB, var.var_rep->index, lower);
  abortIfError(error);
  error = GRBsetdblattrelement(model, GRB_DBL_ATTR_UB, var.var_rep->index, upper);
  abortIfError(error);
}

double GurobiInterface::getVarValue(const Var& var) const {
  double out;
  int error = GRBgetdblattrelement(model, GRB_DBL_ATTR_X, var.var_rep->index, &out);
  abortIfError(error);
  return out;
}
string GurobiInterface::getVarName(const Var& var) const {
  char* value;
  int error = GRBgetstrattrelement(model, GRB_STR_ATTR_VARNAME, var.var_rep->index, &value);
  abortIfError(error);
  return std::string(value);
}

CvxOptStatus GurobiInterface::optimize(){
  int error = GRBoptimize(model);
  abortIfError(error);
  int status;
  GRBgetintattr(model, GRB_INT_ATTR_STATUS, &status);
  if (status == GRB_OPTIMAL) {
    double objval; GRBgetdblattr(model, GRB_DBL_ATTR_OBJVAL, &objval);
    IPI_LOG_INFO("solver objective value: %.3e", objval);
    return CVX_SOLVED;
  }
  else if (status == GRB_INFEASIBLE) return CVX_INFEASIBLE;
  else return CVX_FAILED;
}
CvxOptStatus GurobiInterface::optimizeFeasRelax(){
  double lbpen=GRB_INFINITY, ubpen = GRB_INFINITY,  rhspen=1;
  int error = GRBfeasrelax(model, 0/*sum of viol*/, 0/*just minimize cost of viol*/, &lbpen, &ubpen, &rhspen, NULL);
  abortIfError(error);
  return optimize();
}

void GurobiInterface::setObjective(const AffExpr& expr) {
  GRBdelq(model);

  int nvars;
  GRBgetintattr(model, "NumVars", &nvars);
  assert(nvars == (int)vars.size());

  vector<double> obj(nvars, 0);
  for (size_t i=0; i < expr.size(); ++i) {
    obj[expr.vars[i].var_rep->index] += expr.coeffs[i];
  }
  int error = GRBsetdblattrarray(model, "Obj", 0, nvars, obj.data());
//  vector<int> inds = vars2inds(expr.vars);
//  cout << "inds: " << inds << endl;
//  int error = GRBsetdblattrlist(model, GRB_DBL_ATTR_OBJ, inds.size(),
//      const_cast<int*>(inds.data()),const_cast<double*>(expr.coeffs.data()));
  abortIfError(error);

  GRBsetdblattr(model, "ObjCon", expr.constant);
}

void GurobiInterface::setObjective(const QuadExpr& quad_expr) {
  setObjective(quad_expr.affexpr);
  vector<int> inds1 = vars2inds(quad_expr.vars1);
  vector<int> inds2 = vars2inds(quad_expr.vars2);
  GRBaddqpterms(model, quad_expr.coeffs.size(), const_cast<int*>(inds1.data()),
      const_cast<int*>(inds2.data()), const_cast<double*>(quad_expr.coeffs.data()));
}

void GurobiInterface::writeToFile(const string& fname) {
  int error = GRBwrite(model, fname.c_str());
  abortIfError(error);
}


void GurobiInterface::update() {
  int error = GRBupdatemodel(model);
  abortIfError(error);

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

VarVector GurobiInterface::getVars() const {
  return vars;
}

GurobiInterface::~GurobiInterface() {
  int error = GRBfreemodel(model);
  abortIfError(error);
}

}
}

#endif
