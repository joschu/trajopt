#include <openrave-core.h>
#include <openrave/openrave.h>

#include "trajopt/problem_description.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "sco/expr_op_overloads.hpp"
#include "sco/optimizers.hpp"
#include "quat_ops.hpp"
#include <boost/assign.hpp>
using namespace boost::assign;
using namespace OpenRAVE;
using namespace trajopt;
using namespace sco;
using namespace Eigen;
using namespace std;

typedef BasicArray<AffExpr> AffArray;
typedef BasicArray<Cnt> CntArray;


////// Helper functions ///////////


MatrixXd getTraj(const DblVec& x, const AffArray& arr) {
  MatrixXd out(arr.rows(), arr.cols());
  for (int i=0; i < arr.rows(); ++i) {
    for (int j=0; j < arr.cols(); ++j) {
      out(i,j) = arr(i,j).value(x);
    }
  }
  return out;
}

void AddVarArray(OptProb& prob, int rows, int cols, const string& name_prefix, const VarArray& newvars) {
  vector<VarArray*> arrs(1, &newvars);
  vector<string> prefixes(1, name_prefix);
  vector<int> colss(1, cols);
  AddVarArrays(prob, rows, colss, prefixes, arrs);
}

void AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<string>& name_prefix, const vector<VarArray*>& newvars) {
  int n_arr = name_prefix.size();
  assert(n_arr == new_vars.size());

  vector<MatrixXi> index(n_arr);
  for (int i=0; i < n_arrs; ++i) {
    newvars[i]->resize(rows, cols[i]);
    index[i]->resize(rows, cols[i]);
  }

  vector<string> names;
  int var_idx = prob.getNumVars();
  for (int i=0; i < rows; ++i) {
    for (int k=0; k < n_arr; ++k) {
      for (int j=0; j < cols[k]; ++j) {
        index[k](i,j) = var_idx;
        names.push_back( (boost::format("%s_%i_%i")%name_prefix[k]%i%j).str() );
        ++var_idx;
      }
    }
  }
  prob.createVariables(names); // note that w,r, are both unbounded

  const vector<Var>& vars = prob.getVars();
  for (int i=0; i < rows; ++i) {
    for (int k=0; i < n_arr; ++k) {
      for (int j=0; j < cols[k]; ++j) {
        (*newvars[k])(i,j) = vars[index[i](i,j)];
      }
    }
  }
}

template <typename T> // var or expr
void FiniteDifferences(const BasicArray<T>& x, AffArray& dx, double dt) {
  dx.resize(x.rows()-1, x.cols());
  for (int i=0; i < x.rows(); ++i) {
    for (int j=0; j < x.cols(); ++j) {
      dx(i,j) = x(i+1,j) - x(i,j);
    }
  }
}


//////////////////////////////////////////////

struct ContactTraj {
  double m_dt;
  VarArray m_pA; // local contact point in A
  VarArray m_pB; // local contact point in B
  VarArray m_fAB; // force from B to A
  ContactTraj(OptProb& prob, int n_steps, double dt);
};
typedef boost::shared_ptr<ContactTraj> ContactTrajPtr;

struct RigidBodyTraj {
  double m_dt;
  VarArray m_p; // position trajectory
  MatrixXd m_q; // quaternion trajectory. updated in callback
  VarArray m_r; // rotation increments
  AffArray m_v; // velocity
  AffArray m_a; // acceleration
  AffArray m_w; // angular velocity. updated in callback.
  AffArray m_aa; // angular acceleration. update in callback
  CntArray m_forcecnts;
  RigidBodyTraj(OptProb& prob, int n_steps, double dt);
  void UpdateRotationExpressions(const DblVec& x);
};
typedef boost::shared_ptr<RigidBodyTraj> RigidBodyTrajPtr;


class RigidBodySim : public OptProb {
public:

  double m_dt; // timestep
  vector<RigidBodyTrajPtr> m_rbts;
  vector<ContactTrajPtr> m_cts;
  vector<KinBody::LinkPtr> m_links;
  RigidBodySim(const vector<KinBody::LinkPtr>& links, double dt);
  /**
   * Update integration constraints
   * Update rotation expressions
   */
  void Callback(DblVec& x);

};

ContactTraj::ContactTraj(OptProb& prob, int n_steps, double dt) : m_dt(dt) {
  vector<VarArray*> arrs; arrs += m_pA, m_pB, m_fAB;
  vector<string> prefixes; prefixes += "pA", "pB", "fAB";
  vector<int> cols; cols += 3,3,3;
  AddVarArrays(prob, n_steps, cols, prefixes, arrs);
}

RigidBodyTraj::RigidBodyTraj(OptProb& prob, int n_steps, int dt) : m_dt(dt) {
  vector<VarArray*> arrs; arrs += m_p, m_r;
  vector<string> prefixes; prefixes += "p", "r";
  vector<int> cols; cols += 3,3;
  AddVarArrays(prob, n_steps, cols, prefixes, arrs);
  FiniteDifferences(m_x, m_v, m_dt);
  FiniteDifferences(m_v, m_a, m_dt);
}

void RigidBodyTraj::UpdateRotationExpressions(const DblVec& x) {
  MatrixXd rvals = getTraj(x, m_r);
  // update quaternions
  for (int i=0; i < m_q.rows(); ++i) {
    m_q.row(i) = quatMult(m_q.row(i), quatExp(rvals.row(i)));
  }

  MatrixXd wvals = getW(m_q, m_dt);
  for (int i=0; i < wvals.rows(); ++i) {
    for (int j=0; j < wvals.cols(); ++j) {
      m_w(i,j) = (m_r(i+1,j) - m_r(i,j))/m_dt + wvals(i,j);
    }
  }
  FiniteDifferences(m_w, m_a, m_dt);

}

void RigidBodySim::Callback(DblVec& x) {
  // update rotation expressions
  // set up integration constraints
  for (int i=0; i < m_rbts.size(); ++i) m_rbts[i]->UpdateRotationExpressions(x);
}

RigidBodySim::RigidBodySim(const vector<KinBody::LinkPtr>& links, int n_steps, double dt) : m_links(links), m_dt(dt) {
  for (int i=0; i < links.size(); ++i) {
    m_rbts.push_back(RigidBodyTrajPtr(new RigidBodyTraj(*this, n_steps, dt)));
  }
  for (int i=0; i < links.size(); ++i) {
    for (int j=0; j < links.size(); ++j) {
      m_cts.push_back(ContactTrajPtr(new ContactTraj(links[i], links[j], *this, n_steps, dt)));
    }
  }
}


void Callback(OptProb* prob, DblVec& x) {
  dynamic_cast<RigidBodySim*>(prob)->Callback(x);
}


/**
 * todo:
 * - overlap constraints
 * - complementarity constraints
 * - increment mask
 * - integration constraints
 * - contact point position constraints
 * - friction cone constraints
 * -
 */

int main(int argc, char** argv) {

  int n_steps = 10;
  double dt=.1;



  OptProbPtr prob(new RigidBodySim(xxx));
  prob->setIncrementMask(vector<bool>(xxx);
  prob->addCost(CostPtr(new xxx));
  BasicTrustRegionSQP opt(prob);

  opt.max_iter_ = 50;
  opt.min_approx_improve_ = 1e-10;
  MatrixXd inittraj = MatrixXd::Zero(n_steps, 3);
  opt.initialize(DblVec(inittraj.data(),inittraj.data() + 3*n_steps));
  opt.addCallback(&Callback);
  OptStatus status = opt.optimize();


}
