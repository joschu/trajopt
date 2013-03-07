#include <openrave-core.h>
#include <openrave/openrave.h>

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





/** Error function for discrete Euler Lagrange
 */

MatrixXd CalculateMassMatrix(const RobotAndDOF& rad) {
  todo;
}
vector<MatrixXd> CalculateMassMatrixGrad(const RobotAndDOF& rad) {
  todo;
}



struct DynamicsError : public VectorOfVector {
  RobotAndDOFPtr m_rad;
  double m_dt;
  DynamicsError(RobotAndDOFPtr rad, double dt) : m_rad(rad), m_dt(dt) {
  }
  VectorXd operator()(const VectorXd& x) const {
    int ndof = m_rad->GetDOF();
    VectorXd x0 = x.middleRows(0, ndof),
             tq0 = x.middleRows(ndof, ndof),
             x1 = x.bottomRows(2*ndof, ndof),
             tq1 = x.middleRows(3*ndof, ndof);
    m_rad->SetDOFValues(x0);
    BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
      if (!link->GetGeometries().empty()) {
        moment += link->GetGlobalCOM() * link->GetMass();
        totalmass += link->GetMass();
      }
    m_rad->SetDOFValues(x1);

  }
};

struct DynamicsErrorCalc : public Cost {
  double value(const DblVec& x) const {

  }

};


int main(int argc, char** argv) {

  int n_steps = 10;
  double dt=.1;



  BasicTrustRegionSQP opt(prob);


  OptStatus status = opt.optimize();


}
