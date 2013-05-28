#include <openrave-core.h>
#include <openrave/openrave.h>
#include "trajopt/collision_checker.hpp"
#include "utils/stl_to_string.hpp"
#include "trajopt/common.hpp"
#include "trajopt/problem_description.hpp"
#include "trajopt/collision_terms.hpp"
#include "trajopt/rave_utils.hpp"
#include "osgviewer/osgviewer.hpp"
#include "sco/expr_ops.hpp"
#include <ctime>
#include "utils/eigen_conversions.hpp"
#include "trajopt/trajectory_costs.hpp"
#include "sco/modeling_utils.hpp"
#include "trajopt/plot_callback.hpp"
#include "utils/clock.hpp"
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include "utils/config.hpp"
#include "dynamics_utils.hpp"
#include "quat_ops.hpp"
using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;
using namespace Eigen;
namespace {

bool plotting=false, verbose=false;

#if 0
OR::Transform randomReachablePose(RobotAndDOF& rad, KinBody::LinkPtr link) {
  // todo: save & restore
  DblVec dofvals = rad.RandomDOFValues();
  rad.SetDOFValues(dofvals);
  return link->GetTransform();
}
#endif

}


struct CarError : public VectorOfVector {
  VectorXd operator()(const VectorXd& a) const {
    double x0 = a(0), y0 = a(1), t0 = a(2), x1 = a(3), y1 = a(4), t1 = a(5), L = a(6);
    double ang = (t0+t1)/2;
    return Vector2d(x1 - x0 - L*cos(ang), y1 - y0 - L*sin(ang));
  }
};

struct NormError : public VectorOfVector {
  VectorXd operator()(const VectorXd& a) const {
    VectorXd v(1);
    v[0] =  a.squaredNorm() - 1;
    return v;
  }
};

struct NeedleConfig : public Configuration {
  DblVec m_vals;
  RobotBasePtr m_robot;
  
  NeedleConfig(RobotBasePtr robot) : m_robot(robot) {
    const OR::Transform& T = robot->GetTransform();
    OR::Vector dp = OR::geometry::quatRotate(T.rot, OR::Vector(1,0,0));
    m_vals.resize(6);
    m_vals[0] = T.trans.x;
    m_vals[1] = T.trans.y;
    m_vals[2] = T.trans.z;
    m_vals[3] = dp.x;
    m_vals[4] = dp.y;
    m_vals[5] = dp.z;
  }
  virtual void SetDOFValues(const DblVec& dofs) {
    Vector3d k(dofs[0], dofs[1], dofs[2]);
    k.normalize();
    m_vals = toDblVec(k);
    OR::Vector p(dofs[0], dofs[1], dofs[2]);
    OR::Vector v(dofs[3], dofs[4], dofs[5]);
    OR::Vector q = OR::geometry::quatRotateDirection(OR::Vector(1,0,0), v);
    OR::Transform T(q,p);
    m_robot->SetTransform(T);
  }
  virtual void GetDOFLimits(DblVec& lower, DblVec& upper) const {
    lower.resize(6);
    lower[0] = -INFINITY;
    lower[1] = -INFINITY;
    lower[2] = -INFINITY;
    lower[3] = -1;
    lower[4] = -1;
    lower[5] = -1;
    upper.resize(6);
    upper[0] = INFINITY;
    upper[1] = INFINITY;
    upper[2] = INFINITY;
    upper[3] = 1;
    upper[4] = 1;
    upper[5] = 1;
  }
  virtual DblVec GetDOFValues() {
    return m_vals;
  }
  virtual int GetDOF() const {
    return 6;
  }
  vector<KinBodyPtr> GetBodies() {
    return vector<KinBodyPtr>(1, m_robot);
  }   
  virtual OpenRAVE::EnvironmentBasePtr GetEnv() {
    return m_robot->GetEnv();
  }
  virtual DblMatrix PositionJacobian(int link_ind, const OR::Vector& pt) const {
    PRINT_AND_THROW("not implemented");
  }
  virtual DblMatrix RotationJacobian(int link_ind) const {
    PRINT_AND_THROW("not implemented");    
  }
  virtual bool DoesAffect(const KinBody::Link& link) {
    return link.GetParent() == m_robot;
  }
  virtual std::vector<KinBody::LinkPtr> GetAffectedLinks() {
    return m_robot->GetLinks();
  }
  virtual void GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) {
    links = GetAffectedLinks();
    link_inds.resize(links.size());
    for (int i = 0; i < links.size(); ++i)
      link_inds.push_back(links[i]->GetIndex());    
  }
  virtual DblVec RandomDOFValues() {
    PRINT_AND_THROW("not implemented");
  }
};


inline Vector3d rotVec(const OpenRAVE::Vector& q) {
  return Vector3d(q[1], q[2], q[3]);
}


struct NeedleError : public VectorOfVector {
  IncrementalRBPtr rad0, rad1;
  double radius;
  NeedleError(IncrementalRBPtr rad0, IncrementalRBPtr rad1, double radius) : rad0(rad0), rad1(rad1), radius(radius) {}
  VectorXd operator()(const VectorXd& a) const {
    rad0->SetDOFValues(toDblVec(a.topRows(6)));
    rad1->SetDOFValues(toDblVec(a.middleRows(6,6)));
    OR::Transform Tw0 = rad0->m_body->GetTransform(), Tw1 = rad1->m_body->GetTransform();
    double theta = a(12);
    OR::Transform Ttarg0(OR::geometry::quatFromAxisAngle(OR::Vector(0,0,1), theta),
        OR::Vector(radius*sin(theta), radius*(1-cos(theta)),0));
    OR::Transform pose_err = Tw1.inverse() * Tw0 * Ttarg0;
    VectorXd err = concat(rotVec(pose_err.rot), toVector3d(pose_err.trans));
    return err;
  }
};

template <typename T> 
vector<T> singleton(const T& x) {
  return vector<T>(1,x);
} 

//void(OptProb*, DblVec&)
struct ConfigUpdater {
  vector<IncrementalRBPtr> rbs;
  VarArray vars;
  ConfigUpdater(const vector<IncrementalRBPtr>& rbs, const VarArray& vars) : rbs(rbs), vars(vars) {}
  void update(OptProb*, DblVec& x) {
    MatrixXd rvals = getTraj(x, vars);
    cout << "rvals" << endl << rvals << endl;
    for (int i=0; i < rbs.size(); ++i) {
      rbs[i]->m_q = OR::geometry::quatMultiply(geometry::quatFromAxisAngle(OR::Vector(rvals(i,0), rvals(i,1), rvals(i,2))),rbs[i]->m_q);
      rbs[i]->m_r = OR::Vector(0,0,0);
    }

    setVec(x, vars.flatten(), DblVec(vars.size(), 0));
  }
};


class AngVelCost: public Cost {
public:
  AngVelCost(vector<IncrementalRBPtr> rbs, const VarArray& r, double coeff) :
      rbs_(rbs), r_(r), coeff_(coeff) {
    name_="angvel";
  }
  double value(const DblVec& x) {
    MatrixXd q_(rbs_.size(),4);
    for (int i=0; i < rbs_.size(); ++i) q_.row(i) = toVector4d(rbs_[i]->m_q);
    MatrixXd rvals = getTraj(x, r_);
    MatrixXd qnew(q_.rows(), q_.cols());
    for (int i = 0; i < qnew.rows(); ++i) {
      qnew.row(i) = quatMult(quatExp(rvals.row(i)), q_.row(i));
    }
    MatrixXd wvals = getW(qnew, 1);
    return wvals.array().square().sum()*coeff_;
  }
  ConvexObjectivePtr convex(const DblVec& x, Model* model) {
    MatrixXd q_(rbs_.size(),4);
    for (int i=0; i < rbs_.size(); ++i) q_.row(i) = toVector4d(rbs_[i]->m_q);
    ConvexObjectivePtr out(new ConvexObjective(model));
    MatrixXd wvals = getW(q_, 1);
    for (int i = 0; i < wvals.rows(); ++i) {
      for (int j = 0; j < wvals.cols(); ++j) {
        out->addQuadExpr(exprMult(exprSquare(r_(i + 1, j) - r_(i, j) + wvals(i, j)), coeff_));
      }
    }
    return out;
  }
  vector<IncrementalRBPtr> rbs_;
  VarArray r_;
  double coeff_;
};



int main(int argc, char** argv)
{


  int problem=1;
  {
    Config config;
    config.add(new Parameter<bool>("plotting", &plotting, "plotting"));
    config.add(new Parameter<bool>("verbose", &verbose, "verbose"));
    config.add(new Parameter<int>("problem", &problem, "0: car, 1: needle steering"));
    CommandParser parser(config);
    parser.read(argc, argv);
  }

  RaveInitialize(false, verbose ? Level_Debug : Level_Info);
  EnvironmentBasePtr env = RaveCreateEnvironment();
  env->StopSimulation();
  OSGViewerPtr viewer;
  if (plotting) {
    viewer.reset(new OSGViewer(env));
    viewer->UpdateSceneData();
    env->AddViewer(viewer);
  }

  if (problem == 0) {
    env->Load(string(DATA_DIR) + "/carprob.env.xml");
    RobotBasePtr robot = GetRobot(*env);
    RobotAndDOFPtr rad(new RobotAndDOF(robot, vector<int>(), 11, OR::Vector(0,0,1)));

    int n_steps = 29;
    int n_dof = 3;

    Vector3d start(0,0,0);
    Vector3d goal(0,2,M_PI);

    TrajOptProbPtr prob(new TrajOptProb(n_steps, rad));
    VectorXd vel_coeffs = VectorXd::Ones(3);
    prob->addCost(CostPtr(new JointVelCost(prob->GetVars(), vel_coeffs)));
  
    double lb = (goal - start).norm() / (n_steps-1);
    prob->createVariables(singleton<string>("speed"), singleton<double>(lb),singleton<double>(INFINITY));
    Var lvar = prob->getVars().back();
  
    for (int i=0; i < n_steps-1; ++i) {
      VarVector vars0 = prob->GetVarRow(i), vars1 = prob->GetVarRow(i+1);
      VectorOfVectorPtr f(new CarError());
      VectorXd coeffs = VectorXd::Ones(2);
      VarVector vars = concat(vars0, vars1);  vars.push_back(lvar);
      prob->addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, (boost::format("car%i")%i).str())));
      prob->addCost(CostPtr(new CollisionCost(.01, 10, rad, vars0, vars1)));
    }

    for (int j=0; j < n_dof; ++j) {
      prob->addLinearConstraint(exprSub(AffExpr(prob->GetVar(0,j)), start[j]), EQ);
    }
    for (int j=0; j < n_dof; ++j) {
      prob->addLinearConstraint(exprSub(AffExpr(prob->GetVar(n_steps-1,j)), goal[j]), EQ);
    }


    BasicTrustRegionSQP opt(prob);
    if (plotting) opt.addCallback(PlotCallback(*prob));
  
    MatrixXd initTraj(n_steps, n_dof);  
    for (int idof = 0; idof < n_dof; ++idof) {
      initTraj.col(idof) = VectorXd::LinSpaced(n_steps, start[idof], goal[idof]);
    }
    DblVec initVec = trajToDblVec(initTraj);
    initVec.push_back(lb);
    opt.initialize(initVec);
    
    opt.optimize();
    
  }
  else if (problem == 1) {
    env->Load(string(DATA_DIR) + "/needlebot.xml");
    RobotBasePtr robot = GetRobot(*env);
    ConfigurationPtr rad(new NeedleConfig(robot));

    int n_steps = 19;
    int n_dof = 6;

    TrajOptProbPtr prob(new TrajOptProb(n_steps, rad));

    VectorXd start(n_dof); start << 0,0,0,0,0,0;
    VectorXd goal(n_dof); goal <<  2,.1,.1,0,0,0;

    vector<IncrementalRBPtr> incConfigs;
    for (int i=0; i < n_steps; ++i) {
      incConfigs.push_back(IncrementalRBPtr(new IncrementalRB(robot)));
      prob->setIncremental(prob->GetVars().rblock(i, 3, 3));
    }

    VectorXd vel_coeffs = VectorXd::Ones(3);
    prob->addCost(CostPtr(new JointVelCost(prob->GetVars().block(0,0,n_steps, 3), vel_coeffs)));
    prob->addCost(CostPtr(new AngVelCost(incConfigs, prob->GetVars().block(0,3,n_steps, 3), vel_coeffs[0])));
  
    double radius = 2;
    double lb = 2./n_steps/radius;//(goal - start).norm() / (n_steps-1);
    prob->createVariables(singleton<string>("speed"), singleton<double>(lb),singleton<double>(INFINITY));
    Var lvar = prob->getVars().back();
  

    for (int i=0; i < n_steps-1; ++i) {
      VarVector vars0 = prob->GetVarRow(i), vars1 = prob->GetVarRow(i+1);
      VectorOfVectorPtr f(new NeedleError(incConfigs[i], incConfigs[i+1], radius));
      VectorXd coeffs = VectorXd::Ones(3);
      VarVector vars = concat(vars0, vars1);  vars.push_back(lvar);
      assert(vars.size() == 13);
      prob->addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, (boost::format("neelde%i")%i).str())));
    }

//    for (int i=0; i < n_steps; ++i) {
//      VarVector vars; vars += prob->GetVar(i,3), prob->GetVar(i,4), prob->GetVar(i,5);
//      VectorOfVectorPtr f(new NormError());
//      VectorXd coeffs = VectorXd::Ones(3);
//      prob->addConstraint(ConstraintPtr(new ConstraintFromFunc(f, vars, coeffs, EQ, (boost::format("norm%i")%i).str())));
//    }



    for (int j=0; j < n_dof; ++j) {
      prob->addLinearConstraint(exprSub(AffExpr(prob->GetVar(0,j)), start[j]), EQ);
    }
    for (int j=0; j < n_dof; ++j) {
      prob->addLinearConstraint(exprSub(AffExpr(prob->GetVar(n_steps-1,j)), goal[j]), EQ);
    }

    BasicTrustRegionSQP opt(prob);
    ConfigUpdater updater(incConfigs, prob->GetVars().block(0,3,n_steps,3));
//    opt.addCallback(&Callback);
    opt.addCallback(boost::bind(&ConfigUpdater::update, updater, _1, _2));
    if (plotting) opt.addCallback(PlotCallback(*prob));


    MatrixXd initTraj(n_steps, n_dof);  
    for (int idof = 0; idof < n_dof; ++idof) {
      initTraj.col(idof) = VectorXd::LinSpaced(n_steps, start[idof], goal[idof]);
    }
    DblVec initVec = trajToDblVec(initTraj);
    initVec.push_back(lb);
    opt.initialize(initVec);
    
    opt.optimize();
    
  }






  RaveDestroy();


}
