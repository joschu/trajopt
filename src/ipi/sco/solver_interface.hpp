#pragma once
#include <boost/shared_ptr.hpp>
#include <vector>
#include <string>
#include "sco_fwd.hpp"
#include <iosfwd>
#include <limits>
/**
@file solver_interface.hpp
@brief Interface to convex solvers

  This is based on Gurobi's nice c++ API (though the SCO Gurobi backend uses the Gurobi c api).
  However, our intention is to allow for different solvers to be used as backends.
 */

namespace ipi{
namespace sco{

using std::string;
using boost::shared_ptr;
using std::vector;
using std::ostream;

enum CvxOptStatus {
  CVX_SOLVED,
  CVX_INFEASIBLE,
  CVX_FAILED
};

typedef vector<Var> VarVector;
typedef vector<AffExpr> AffExprVector;
typedef vector<QuadExpr> QuadExprVector;

/**
 * Convex optimization problem
 *
 */
class Model {
public:
  virtual Var addVar(const string& name)=0;
  virtual Var addVar(const string& name, double lb, double ub);

  virtual Cnt addEqCnt(const AffExpr&, const string& name)=0; // expr == 0
  virtual Cnt addIneqCnt(const AffExpr&, const string& name)=0; // expr <= 0
  virtual Cnt addIneqCnt(const QuadExpr&, const string& name)=0; // expr <= 0

  virtual void removeVar(const Var& var) = 0;
  virtual void removeCnt(const Cnt& cnt) = 0;
  virtual void removeVars(const VarVector& vars);
  virtual void removeCnts(const vector<Cnt>& cnts);

  virtual void update() = 0; // call after adding/deleting stuff
  virtual void setVarBounds(const Var& var, double lower, double upper)=0;
  virtual double getVarValue(const Var& var) const=0;
  virtual vector<double> getVarValues(const VarVector& vars) const;
  virtual string getVarName(const Var&) const = 0;
  virtual CvxOptStatus optimize()=0;

  virtual void setObjective(const AffExpr&)=0;
  virtual void setObjective(const QuadExpr&)=0;
  virtual void writeToFile(const string& fname)=0;

  virtual VarVector getVars() const=0;

  virtual ~Model() {}

};

struct VarRep {
  VarRep(int index, Model* si) : index(index), removed(false), solver_interface(si) {}
  int index;
  bool removed;
  Model* solver_interface;
};

struct Var {
  VarRep* var_rep;
  Var() : var_rep(NULL) {}
  Var(VarRep* var_rep) : var_rep(var_rep) {}
  Var(const Var& other) : var_rep(other.var_rep) {}
  double value() const {return var_rep->solver_interface->getVarValue(*this);}
  double value(const double* x) const {return x[var_rep->index];}
  double value(const vector<double>& x) const {assert(var_rep->index < (int)x.size()); return x[var_rep->index];}
};

struct CntRep {
  CntRep(int index, Model* si) : index(index), removed(false), solver_interface(si){}
  int index;
  bool removed;
  Model* solver_interface;
};

struct Cnt {
  CntRep* cnt_rep;
  Cnt(): cnt_rep(NULL) {}
  Cnt(CntRep* cnt_rep) : cnt_rep(cnt_rep) {}
  Cnt(const Cnt& other) : cnt_rep(other.cnt_rep) {}
};

struct AffExpr { // affine expression
  double constant;
  vector<double> coeffs;
  vector<Var> vars;
  AffExpr() : constant(0) {}
  explicit AffExpr(double a) : constant(a) {}
  explicit AffExpr(const Var& v) : constant(0), coeffs(1, 1), vars(1, v) {}
  AffExpr(const AffExpr& other) :
    constant(other.constant), coeffs(other.coeffs), vars(other.vars) {}
  size_t size() const {return coeffs.size();}
  double value() const;
  double value(const double* x) const;
  double value(const vector<double>& x) const;
};

struct QuadExpr {
  AffExpr affexpr;
  vector<double> coeffs;
  vector<Var> vars1;
  vector<Var> vars2;
  QuadExpr() {}
  explicit QuadExpr(double a) : affexpr(a) {}
  explicit QuadExpr(const Var& v) : affexpr(v) {}
  explicit QuadExpr(const AffExpr& aff) : affexpr(aff) {}
  size_t size() const {return coeffs.size();}
  double value() const;
  double value(const double* x) const;
  double value(const vector<double>& x) const;
};


ostream& operator<<(ostream&, const Var&);
ostream& operator<<(ostream&, const Cnt&);
ostream& operator<<(ostream&, const AffExpr&);
ostream& operator<<(ostream&, const QuadExpr&);


enum CvxSolverID {
  SOLVER_GUROBI,
  SOLVER_CUSTOM
};
ModelPtr createModel(CvxSolverID);

}}
