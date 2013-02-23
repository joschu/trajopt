#include <openrave-core.h>
#include <openrave/openrave.h>

#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "ipi/sco/expr_op_overloads.hpp"
#include "ipi/sco/optimizers.hpp"
using namespace OpenRAVE;
using namespace trajopt;
using namespace ipi::sco;
using namespace Eigen;
using namespace std;

typedef BasicArray<AffExpr> AffArray;

string data_dir() {
  string out = DATA_DIR;
  return out;
}

Vector4d quatMult(const Vector4d& q1, const Vector4d& q2) {
  return Vector4d(
      q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
      q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
      q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3],
      q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1]);
}

DblMatrix getTraj(const DblVec& x, const AffArray& arr) {
  DblMatrix out(arr.rows(), arr.cols());
  for (int i=0; i < arr.rows(); ++i) {
    for (int j=0; j < arr.cols(); ++j) {
      out(i,j) = arr(i,j).value(x);
    }
  }
  return out;
}

Vector4d quatExp(const Vector3d& r) {
  // see http://www.lce.hut.fi/~ssarkka/pub/quat.pdf
  double normr = r.norm();
  if (normr > 1e-10) {
    Vector4d q;
    q(0) = cos(normr / 2);
    q.bottomRows(3) = (r/normr) * sin(normr/2);
    return q;
  }
  else return Vector4d(1,0,0,0);
}

Vector3d quatLog(const Vector4d& q) {
  Vector3d v = q.bottomRows(3);
  double s = q(0);
  Vector3d out = (acos(s) / v.norm()) * v;
  return out;
}
Vector4d quatInv(const Vector4d& q) {
  Vector4d qinv = q;
  qinv.bottomRows(3) *= -1;
  return qinv;
}

DblMatrix getW(const DblMatrix& qs, double dt) {
  DblMatrix out(qs.rows()-1, 3);
  for (int i=0; i < out.rows(); ++i) {
    out.row(i) = (2/dt)*quatLog(quatMult(quatInv(qs.row(i)), (qs.row(i+1))));    
  }
  // cout << "qs: " << endl << qs << endl;
  return out;
}

class QuatTrajProb : public OptProb {
public:
  DblMatrix q_; // Tx4 quaternion trajectory
  double dt_; // timestep
  VarArray r_; // Tx3 trajectory increment variables
  // AffArray w_; // Tx3 velocities variables
  BasicArray<Cnt> cnts_;
  QuatTrajProb(const DblMatrix& q, int n_steps, float dt);
  int GetNumSteps() {return q_.rows();}  
  void Callback(DblVec& x); // update integration constraints
                            // update quaternions, project out integration error
};
typedef boost::shared_ptr<QuatTrajProb> QuatTrajProbPtr;



void QuatTrajProb::Callback(DblVec& x) {
  ///// Reparametrize problem

  DblMatrix rvals = getTraj(x, r_);
  // update quaternions
  for (int i=0; i < q_.rows(); ++i) {
    q_.row(i) = quatMult(q_.row(i), quatExp(rvals.row(i)));    
  }

#if 0
  // project out integration error
  for (int i=0; i < w_.rows(); ++i) {
    Vector3d wproj = (2/dt_)*quatLog(quatMult(quatInv(q_.row(i)), (q_.row(i+1))));
    cout << i << " wproj " << wproj.transpose() << endl;
    for (int j=0; j < 3; ++j) {
      x[w_(i,j).var_rep->index] = wproj(j); // get rid of integration error
    }
  }
  for (int i=0; i < r_.rows(); ++i) {
    for (int j=0; j < 3; ++j) {
      x[r_(i,j).var_rep->index] = 0; // set increment back to zero
    }
  }
  
  
  
  ////// Update integration constraints /////
  
  if (cnts_.rows() == 0) { // create constraint array
    cnts_.resize(GetNumSteps()-1, 3);      
  }
  else { // delete all constraints in array
    for (int i=0; i < cnts_.rows(); ++i) {
      for (int j=0; j < cnts_.cols(); ++j) {
        model_->removeCnt(cnts_(i,j));
      }
    }
  }  
  // make new constraints
  DblMatrix wvals = getTraj(x, w_);
  for (int i=0; i < cnts_.rows(); ++i) { // create all new constraints
    for (int j=0; j < cnts_.cols(); ++j) {
      cnts_(i,j) = model_->addEqCnt( AffExpr(wvals(i,j) * dt_) + r_(i+1,j) - r_(i,j) - w_(i,j) * dt_  , "");
    }
  }


  cout << "r vals: " << endl << rvals << endl;
  cout << "w vals: " << endl << wvals << endl;
  cout << "q after\n" << q_ << endl;
  #endif
}

QuatTrajProb::QuatTrajProb(const DblMatrix& q, int n_steps, float dt) : q_(q), dt_(dt) {

  r_.resize(n_steps, 3); 
    
  BasicArray<int> rindex(n_steps, 3), windex(n_steps, 3); // save indices in the variable vector
    
  vector<string> names;
  for (int i=0; i < n_steps; ++i) {
    for (int j=0; j < 3; ++j) {
      rindex(i,j) = names.size();
      names.push_back( (boost::format("r_%i_%i")%i%j).str() );
      // if (i < n_steps - 1) {
      //   windex(i,j) = names.size();
      //   names.push_back( (boost::format("w_%i_%i")%i%j).str() );
      // }
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

class AngVelCost : public Cost {
public:
  AngVelCost(const DblMatrix& q, const VarArray& r, double dt) : q_(q), r_(r), dt_(dt) {}
  double value(const DblVec& x) {
    DblMatrix rvals = getTraj(x, r_);
    DblMatrix qnew(q_.rows(), q_.cols());
    for (int i=0; i < qnew.rows(); ++i) {
      qnew.row(i) = quatMult(q_.row(i), quatExp(rvals.row(i)));
    }
    DblMatrix wvals = getW(qnew, dt_);
    return wvals.array().square().sum();
  }
  ConvexObjectivePtr convex(const DblVec& x, Model* model) {
    ConvexObjectivePtr out(new ConvexObjective(model));
    DblMatrix wvals = getW(q_, dt_);    
    for (int i=0; i < wvals.rows(); ++i) {
      for (int j=0; j < wvals.cols(); ++j) {        
        out->addQuadExpr(exprSquare( (r_(i+1,j) - r_(i,j))*(1/dt_)  + wvals(i,j) ));
      }
    }
    return out;
  }
  const DblMatrix& q_;
  VarArray r_;
  double dt_;
};

#if 0
template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}
#endif


void Callback(OptProb* prob, DblVec& x) {
  dynamic_cast<QuatTrajProb*>(prob)->Callback(x);
}

int main(int argc, char** argv) {

#if 0  
  RaveInitialize(false, Level_Info);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();

  ALWAYS_ASSERT(env->Load(data_dir() + "/box.xml"));
  ALWAYS_ASSERT(env->Load(data_dir() + "/boxbot.xml"));
  
  RobotAndDOFPtr rad(new RobotAndDOF(boxbot, IntVec(), DOF_Transform));
#endif    

  int n_steps = 10;
  double dt=.1;

  
  OpenRAVE::Vector qstartrave(1,0,0,0), qendrave(0,1,0,0);
  VectorXd t = VectorXd::LinSpaced(n_steps, 0, 1);
  
  DblMatrix slerptraj(n_steps, 4);
  for (int i=0; i < n_steps; ++i) {
    slerptraj.row(i) = toVector4d(geometry::quatSlerp(qstartrave, qendrave, t[i]));
  }
  
  cout << "slerp traj: " << endl << slerptraj << endl;

  DblMatrix noisyquats = slerptraj;
  noisyquats.middleRows(1, n_steps-2) += DblMatrix::Random(n_steps-2, noisyquats.cols()) * .1;
  for (int i=0; i < noisyquats.rows(); ++i) noisyquats.row(i) /= noisyquats.row(i).norm();
  cout << "noisy traj: " << endl <<  noisyquats << endl;
  
  
  QuatTrajProbPtr prob(new QuatTrajProb(noisyquats, n_steps, dt));
  prob->setIncrementMask(vector<bool>(prob->getNumVars(), true));
  prob->addCost(CostPtr(new AngVelCost(prob->q_, prob->r_, dt)));
  BasicTrustRegionSQP opt(prob);
  
  
  opt.max_iter_ = 50;
  opt.min_approx_improve_ = 1e-10;
  DblMatrix inittraj = DblMatrix::Zero(n_steps, 3);
  opt.initialize(DblVec(inittraj.data(),inittraj.data() + 3*n_steps));
  opt.addCallback(&Callback);
  OptStatus status = opt.optimize();
  
  
  cout << "slerp cost: " << getW(slerptraj, dt).array().square().sum() << endl;
  cout << "cost from optimization: " << opt.results().total_cost << endl;
  
}