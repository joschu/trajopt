#include "solver_interface.hpp"
#include <iostream>
#include <boost/foreach.hpp>
using namespace std;

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


Var Model::addVar(const string& name, double lb, double ub) {
  Var v = addVar(name);
  setVarBounds(v, lb, ub);
  return v;
}
void Model::removeVar(const Var& var) {
  vector<Var> vars(1,var);
  removeVars(vars);
}
void Model::removeCnt(const Cnt& cnt) {
  vector<Cnt> cnts(1, cnt);
  removeCnts(cnts);
}

double Model::getVarValue(const Var& var) const {
  VarVector vars(1,var);
  return getVarValues(vars)[0];
}

void Model::setVarBounds(const Var& var, double lower, double upper) {
  vector<double> lowers(1,lower), uppers(1, upper);
  vector<Var> vars(1,var);
  setVarBounds(vars, lowers, uppers);
}


ostream& operator<<(ostream& o, const Var& v) {
  if (v.var_rep != NULL)
    o << v.var_rep->name;
  else
    o << "nullvar";
  return o;
}
ostream& operator<<(ostream& o, const Cnt& c) {
  o << c.cnt_rep->expr << ((c.cnt_rep->type == EQ) ? " == 0" : " <= 0");
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
  // extern ModelPtr createGurobiModel();
  // return createGurobiModel();
  extern ModelPtr createBPMPDModel();
  return createBPMPDModel();
}



}

