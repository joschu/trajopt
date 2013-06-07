#include <openrave-core.h>
#include <openrave/openrave.h>
#include "o3.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "sco/expr_op_overloads.hpp"
#include "sco/optimizers.hpp"
#include "quat_ops.hpp"
using namespace OpenRAVE;
using namespace trajopt;
using namespace sco;
using namespace Eigen;
using namespace std;

typedef BasicArray<AffExpr> AffArray;

string data_dir() {
  string out = DATA_DIR;
  return out;
}


MatrixXd getTraj(const DblVec& x, const AffArray& arr) {
  MatrixXd out(arr.rows(), arr.cols());
  for (int i=0; i < arr.rows(); ++i) {
    for (int j=0; j < arr.cols(); ++j) {
      out(i,j) = arr(i,j).value(x);
    }
  }
  return out;
}


class QuatTrajProb : public OptProb {
public:
  MatrixXd q_; // Tx4 quaternion trajectory
  double dt_; // timestep
  VarArray r_; // Tx3 trajectory increment variables
  // AffArray w_; // Tx3 velocities variables
  QuatTrajProb(const MatrixXd& q, int n_steps, float dt);
  int GetNumSteps() {return q_.rows();}  
  void Callback(DblVec& x); // update integration constraints
                            // update quaternions, project out integration error
};
typedef boost::shared_ptr<QuatTrajProb> QuatTrajProbPtr;



void QuatTrajProb::Callback(DblVec& x) {
  ///// Reparametrize problem

  MatrixXd rvals = getTraj(x, r_);
  // update quaternions
  for (int i=0; i < q_.rows(); ++i) {
    q_.row(i) = quatMult(q_.row(i), quatExp(rvals.row(i)));    
  }

}

QuatTrajProb::QuatTrajProb(const MatrixXd& q, int n_steps, float dt) : q_(q), dt_(dt) {

  r_.resize(n_steps, 3); 
    
  BasicArray<int> rindex(n_steps, 3), windex(n_steps, 3); // save indices in the variable vector
    
  vector<string> names;
  for (int i=0; i < n_steps; ++i) {
    for (int j=0; j < 3; ++j) {
      rindex(i,j) = names.size();
      names.push_back( (boost::format("r_%i_%i")%i%j).str() );
    }
  }
  createVariables(names); // note that w,r, are both unbounded

  const vector<Var>& vars = getVars();
  for (int i=0; i < n_steps; ++i) {
    for (int j=0; j < 3; ++j) {
      r_(i,j) = vars[rindex(i,j)];
      // if (i < n_steps - 1) w_(i,j) = vars[windex(i,j)];
    }
  }
  
  // set start and end fixed
  for (int j=0; j < 3; ++j) {
    model_->addEqCnt(AffExpr(r_(0,j)),"");
    model_->addEqCnt(AffExpr(r_(n_steps-1,j)),"");
  }

}


void Callback(OptProb* prob, DblVec& x) {
  dynamic_cast<QuatTrajProb*>(prob)->Callback(x);
}

int main(int argc, char** argv) {

  int n_steps = 10;
  double dt=.1;

  
  OpenRAVE::Vector qstartrave(1,0,0,0), qendrave(0,1,0,0);
  VectorXd t = VectorXd::LinSpaced(n_steps, 0, 1);
  
  MatrixXd slerptraj(n_steps, 4);
  for (int i=0; i < n_steps; ++i) {
    slerptraj.row(i) = toVector4d(geometry::quatSlerp(qstartrave, qendrave, t[i]));
  }
  
  cout << "slerp traj: " << endl << slerptraj << endl;

  MatrixXd noisyquats = slerptraj;
  noisyquats.middleRows(1, n_steps-2) += MatrixXd::Random(n_steps-2, noisyquats.cols()) * .1;
  for (int i=0; i < noisyquats.rows(); ++i) noisyquats.row(i) /= noisyquats.row(i).norm();
  cout << "noisy traj: " << endl <<  noisyquats << endl;
  
  
  QuatTrajProbPtr prob(new QuatTrajProb(noisyquats, n_steps, dt));
  prob->setIncrementMask(vector<bool>(prob->getNumVars(), true));
  prob->addCost(CostPtr(new AngVelCost(prob->q_, prob->r_, dt)));
  BasicTrustRegionSQP opt(prob);
  
  
  opt.max_iter_ = 50;
  opt.min_approx_improve_ = 1e-10;
  MatrixXd inittraj = MatrixXd::Zero(n_steps, 3);
  opt.initialize(DblVec(inittraj.data(),inittraj.data() + 3*n_steps));
  opt.addCallback(&Callback);
  opt.optimize();
  
  
  cout << "slerp cost: " << getW(slerptraj, dt).array().square().sum() << endl;
  cout << "cost from optimization: " << opt.results().total_cost << endl;
  
}
