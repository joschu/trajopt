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
//double QuadExpr::value() const {
//  double out = affexpr.value();
//  for (size_t i=0; i < size(); ++i) {
//    out += coeffs[i] * vars1[i].value() * vars2[i].value();
//  }
//  return out;
//}

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

void Model::setVarBounds(const VarVector& vars, const vector<double>& lower, const vector<double>& upper) {
  for (int i=0; i < vars.size(); ++i) setVarBounds(vars[i], lower[i], upper[i]);
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
  extern ModelPtr createGurobiModel();
  return createGurobiModel();
}



}

