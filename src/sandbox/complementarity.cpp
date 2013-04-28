#include "complementarity.hpp"
#include "sco/expr_ops.hpp"
#include "utils/eigen_conversions.hpp"
using namespace Eigen;

namespace sco {

DblVec ComplCnt::value(const DblVec& x) {
  VectorXd xf = getVec(x, fvars_);
  VectorXd xg = getVec(x, gvars_);
  return util::toDblVec(f_->call(xf).cwiseProduct(g_->call(xg)));
}
ConvexConstraintsPtr ComplCnt::convex(const DblVec& x, Model* model) {
  // f.g + f.dg + g.df
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  VectorXd xf = getVec(x, fvars_);
  VectorXd xg = getVec(x, gvars_);
  VectorXd fvals = f_->call(xf);    
  VectorXd gvals = g_->call(xg);
  int k = fvals.size();
  assert (gvals.size() == k);
  MatrixXd fgrad = fgrad_->call(xf);
  MatrixXd ggrad = ggrad_->call(xg);
  for (int i=0; i < k; ++i) {
    AffExpr fexpri = affFromValGrad(fvals(i), xf, fgrad.row(i), fvars_);
    AffExpr gexpri = affFromValGrad(gvals(i), xg, ggrad.row(i), gvars_);
    AffExpr fgexpr;
    exprInc(fgexpr, exprMult(fexpri, gvals(i)));
    exprInc(fgexpr, exprMult(gexpri, fvals(i)));
    exprDec(fgexpr, fvals(i)*gvals(i));
    if (type_ == EQ)
      out->addEqCnt(fgexpr);      
    else
      out->addIneqCnt(fgexpr);
  }
  return out;
}

}