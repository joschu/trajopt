#include "solver_interface.hpp"
#include <iostream>
#include <boost/foreach.hpp>
#include "ipi/logging.hpp"
using namespace std;

namespace ipi {
namespace sco {

double AffExpr::value(const double* x) const {
  double out = constant;
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars[i].value(x);
  }
  return out;
}
double AffExpr::value(const vector<double>& x) const {
  double out = constant;
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars[i].value(x);
  }
  return out;
}
double AffExpr::value() const {
  assert(coeffs.size() == vars.size());
  double out = constant;
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars[i].value();
  }
  return out;
}
double QuadExpr::value(const vector<double>& x) const {
  double out = affexpr.value(x);
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars1[i].value(x) * vars2[i].value(x);
  }
  return out;
}
double QuadExpr::value(const double* x) const {
  double out = affexpr.value(x);
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars1[i].value(x) * vars2[i].value(x);
  }
  return out;
}
double QuadExpr::value() const {
  double out = affexpr.value();
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars1[i].value() * vars2[i].value();
  }
  return out;
}

Var Model::addVar(const string& name, double lb, double ub) {
  Var v = addVar(name);
  setVarBounds(v, lb, ub);
  return v;
}
void Model::removeVars(const VarVector& vars) {
  BOOST_FOREACH(const Var& var, vars) removeVar(var);
}
void Model::removeCnts(const vector<Cnt>& cnts) {
  BOOST_FOREACH(const Cnt& cnt, cnts) removeCnt(cnt);
}
vector<double> Model::getVarValues(const VarVector& vars) const {
  vector<double> out(vars.size());
  for (size_t i=0; i < out.size(); ++i) out[i] = getVarValue(vars[i]);
  return out;
}




ostream& operator<<(ostream& o, const Var& v) {
  if (v.var_rep != NULL)
    o << v.var_rep->solver_interface->getVarName(v);
  else
    o << "nullvar";
  return o;
}
ostream& operator<<(ostream& o, const Cnt& c) {
  IPI_ABORT("NOT IMPLEMENTED");
  return o;
}
ostream& operator<<(ostream& o, const AffExpr& e) {
  o << e.constant;
  for (size_t i=0; i < e.size(); ++i) {
    o << " + " << e.coeffs[i] << "*" << e.vars[i];
  }
  return o;
}
ostream& operator<<(ostream& o, const QuadExpr& e) {
  o << e.affexpr;
  for (size_t i=0; i < e.size(); ++i) {
    o << " + " << e.coeffs[i] << "*" << e.vars1[i] << "*" << e.vars2[i];
  }
  return o;
}


ModelPtr createModel(CvxSolverID solver) {
  if (solver == SOLVER_GUROBI) {
#ifdef HAVE_GUROBI
    extern ModelPtr createGurobiInterface();
    return createGurobiInterface();
#else
    IPI_ABORT("DON'T HAVE GUROBI INTERFACE");
#endif
  }
  else {
    IPI_ABORT("DON'T HAVE GUROBI INTERFACE");
  }
  return ModelPtr();
}



}
}
