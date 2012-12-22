#include "solver_interface.hpp"

struct _GRBmodel;
typedef struct _GRBmodel GRBmodel;

namespace ipi {
namespace sco {

class GurobiInterface : public Model {
public:
  GRBmodel* model;
  vector<Var> vars;
  vector<Cnt> cnts;

  GurobiInterface();

  Var addVar(const string& name);
  Var addVar(const string& name, double lower, double upper);

  Cnt addEqCnt(const AffExpr&, const string& name);
  Cnt addIneqCnt(const AffExpr&, const string& name);
  Cnt addIneqCnt(const QuadExpr&, const string& name);

  void removeVar(const Var&);
  void removeCnt(const Cnt&);

  void update();

  void setVarBounds(const Var&, double lower, double upper);

  double getVarValue(const Var&) const;
  string getVarName(const Var&) const;
  CvxOptStatus optimize();
  CvxOptStatus optimizeFeasRelax();

  void setObjective(const AffExpr&);
  void setObjective(const QuadExpr&);
  void writeToFile(const string& fname);

  VarVector getVars() const;

  ~GurobiInterface();

};


}
}

