#pragma once
#include "sco/modeling_utils.hpp"


namespace sco {

  struct ComplCnt : public Constraint {
    ConstraintType type_;
    VectorOfVectorPtr f_, g_;
    MatrixOfVectorPtr fgrad_, ggrad_;
    VarVector fvars_, gvars_;
    ComplCnt(ConstraintType type, VectorOfVectorPtr f, VectorOfVectorPtr g, const VarVector& fvars, const VarVector& gvars, double epsilon) 
    : type_(type), f_(f), g_(g), fgrad_(forwardNumJac(f, epsilon)), ggrad_(forwardNumJac(g, epsilon)), fvars_(fvars), gvars_(gvars) {}
    ComplCnt(ConstraintType type, VectorOfVectorPtr f, VectorOfVectorPtr g, MatrixOfVectorPtr fgrad, MatrixOfVectorPtr ggrad, const VarVector& fvars, const VarVector& gvars)
    : type_(type), f_(f), g_(g), fgrad_(fgrad), ggrad_(ggrad), fvars_(fvars), gvars_(gvars) {}  
    DblVec value(const DblVec& x);
    ConvexConstraintsPtr convex(const DblVec& x, Model* model);
    ConstraintType type() {return type_;}
  };

}
