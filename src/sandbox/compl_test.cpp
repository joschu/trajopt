#include "complementarity.hpp"
#include "boost/assign.hpp"
#include "utils/stl_to_string.hpp"
#include "boost/format.hpp"
#include "sco/optimizers.hpp"

using namespace Eigen;
using namespace boost::assign;
using namespace std;
using namespace sco;
void setupProblem(OptProbPtr& probptr, size_t nvars) {
  probptr.reset(new OptProb());
  vector<string> var_names;
  for (size_t i=0; i < nvars; ++i) {
    var_names.push_back( (boost::format("x_%i")%i).str() );
  }
  probptr->createVariables(var_names);
}


struct DistFromPt : public ScalarOfVector {
  VectorXd pt_;
  DistFromPt(const Eigen::VectorXd& pt) : pt_(pt) {}
  double operator()(const VectorXd& x) const {
    return (x-pt_).squaredNorm();
  }
};

struct g0 : public VectorOfVector {
  VectorXd operator()(const VectorXd& x) const {
    return x;
  }
};

struct h0 : public VectorOfVector {
  VectorXd operator()(const VectorXd& x) const {
    return x;
    
  }
};


int main(int argc, char** argv) {
  
  OptProbPtr prob;
  setupProblem(prob, 2);

  DblVec lb(2, 0);
  prob->setLowerBounds(lb);

  VarVector x0 = list_of(prob->getVars()[0]);
  VarVector y0 = list_of(prob->getVars()[1]);
  

  prob->addCost(CostPtr(new CostFromFunc(ScalarOfVectorPtr(new DistFromPt(Vector2d(2,1))), prob->getVars(), "distfunc")));
  prob->addConstraint(ConstraintPtr(new ComplCnt(INEQ, VectorOfVectorPtr(new g0()), VectorOfVectorPtr(new h0()), x0, y0, 1e-4)));
  
  BasicTrustRegionSQP solver(prob);
  vector<double> x = list_of(1)(1);
  solver.initialize(x);
  OptStatus status = solver.optimize();
  cout << "status: " << status << endl;
  cout << CSTR(solver.x()) << endl;
  
    
}