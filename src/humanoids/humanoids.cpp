#include "humanoids.hpp"
#include "sco/modeling_utils.hpp"
#include "sco/expr_ops.hpp"
#include "sco/expr_vec_ops.hpp"
#include "sco/expr_op_overloads.hpp"
#include <boost/foreach.hpp>
#include "utils/eigen_conversions.hpp"
#include "trajopt/problem_description.hpp"
#include "humanoids/hull2d.hpp"
using namespace util;
using namespace Eigen;
using namespace std;
using namespace Json;
namespace trajopt {

/**
import openravepy
env = openravepy.Environment()
env.Load("atlas.xml")
robot = env.GetRobots()[0]
rfoot = robot.GetLink("r_foot")
aabb = rfoot.ComputeLocalAABB(rfoot)
aabb.pos()+aabb.extents()
aabb.pos()-aabb.extents()
 */
const double footpolydata[8] = {
    0.17939, 0.062707,
    0.17939, -0.061646,
    -0.08246, -0.061646,
    -0.08246, 0.061646};
MatrixX2d local_aabb_poly = Map<const MatrixX2d>(footpolydata,4,2);

/**
px,py,pz=aabb.pos()
ex,ey,ez=aabb.extents()
np.array([(px+xsgn*ex,py+ysgn*ey,pz) for xsgn in (-1,1) for ysgn in (-1,1)])
 *
 */
const double footbottomdata[12] = {
    0.17939  ,  0.062707 , -0.0353015,
    0.17939  , -0.061646 , -0.0353015,
    -0.08246  , -0.061646 , -0.0353015,
    -0.08246  ,  0.062707 , -0.0353015
};
MatrixX3d footbottompoints = Map<const MatrixX3d>(footbottomdata, 4, 3);


void PolygonToEquations(const MatrixX2d& pts, MatrixX2d& ab, VectorXd& c) {
  // ax + by + c <= 0
  // assume polygon is convex
  ab.resize(pts.rows(),2);
  c.resize(pts.rows());
  Vector2d p0 = pts.row(0);

  for (int i=0; i < pts.rows(); ++i) {
    int i1 = (i+1) % pts.rows();
    double x0 = pts(i,0),
        y0 = pts(i,1),
        x1 = pts(i1,0),
        y1 = pts(i1,1);
    ab(i,0) = -(y1 - y0);
    ab(i,1) = x1 - x0;
    ab.row(i).normalize();
    c(i) = -ab.row(i).dot(pts.row(i));
  }

  Vector2d centroid = pts.colwise().mean();
  if (ab.row(0) * centroid + c(0) >= 0) {
    ab *= -1;
    c *= -1;
  }

}


ZMPConstraint::ZMPConstraint(RobotAndDOFPtr rad, const MatrixX2d& hullpts, const VarVector& vars) :
          m_rad(rad), m_vars(vars), m_pts(hullpts) {

  // find the equations representing the polygon
  PolygonToEquations(m_pts, m_ab, m_c);
}

DblVec ZMPConstraint::value(const DblVec& x) {
  m_rad->SetDOFValues(getDblVec(x,m_vars));
  OR::Vector moment(0,0,0);
  // calculate center of mass
  float totalmass = 0;
  BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
    if (!link->GetGeometries().empty()) {
      moment += link->GetGlobalCOM() * link->GetMass();
      totalmass += link->GetMass();
    }
  }
  moment /= totalmass;
  Vector2d xy;
  xy(0) = moment.x;
  xy(1) = moment.y;
  return toDblVec(m_ab * xy + m_c);
}

ConvexConstraintsPtr ZMPConstraint::convex(const DblVec& x, Model* model) {
  DblVec curvals = getDblVec(x, m_vars);
  m_rad->SetDOFValues(curvals);
  DblMatrix jacmoment = DblMatrix::Zero(3, curvals.size());
  OR::Vector moment(0,0,0);
  float totalmass = 0;
  BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
    if (!link->GetGeometries().empty()) {
      OR::Vector cm = link->GetGlobalCOM();
      moment += cm * link->GetMass();
      jacmoment += m_rad->PositionJacobian(link->GetIndex(), cm) * link->GetMass();
      totalmass += link->GetMass();
    }
  }
  moment /= totalmass;
  jacmoment /= totalmass;

  AffExpr x_expr = AffExpr(moment.x) + varDot(jacmoment.row(0), m_vars) - jacmoment.row(0).dot(toVectorXd(curvals));
  AffExpr y_expr = AffExpr(moment.y) + varDot(jacmoment.row(1), m_vars) - jacmoment.row(1).dot(toVectorXd(curvals));

  ConvexConstraintsPtr out(new ConvexConstraints(model));
  for (int i=0; i < m_ab.rows(); ++i) {
    out->addIneqCnt(m_ab(i,0) * x_expr + m_ab(i,1) * y_expr + m_c(i));
  }
  return out;
}

void ZMPConstraint::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  Matrix<float, Dynamic, 3, RowMajor> xyz(m_ab.rows()*2, 3);
  xyz.col(2).setZero();
  int npts = m_pts.rows();
  for (int i=0; i < npts; ++i) {
    xyz(2*i,0) = m_pts(i,0);
    xyz(2*i,1) = m_pts(i,1);
    xyz(2*i+1,0) = m_pts((i+1)%npts,0);
    xyz(2*i+1,1) = m_pts((i+1)%npts,1);
  }
  handles.push_back(env.drawlinelist(xyz.data(), xyz.rows(), sizeof(float)*3, 4, OR::RaveVector<float>(1,0,0,1)));

  m_rad->SetDOFValues(getDblVec(x,m_vars));
  OR::Vector moment(0,0,0);
  float totalmass = 0;
  BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
    if (!link->GetGeometries().empty()) {
      moment += link->GetGlobalCOM() * link->GetMass();
      totalmass += link->GetMass();
    }
  }
  moment /= totalmass;
  OR::Vector moment_ground = moment; moment_ground.z = 0;
  handles.push_back(env.drawarrow(moment, moment_ground, .005, OR::RaveVector<float>(1,0,0,1)));
}

struct StaticTorqueCostCalc : public VectorOfVector {
  RobotAndDOFPtr m_rad;
  StaticTorqueCostCalc(const RobotAndDOFPtr rad) : m_rad(rad) {}
  VectorXd operator()(const VectorXd& x) const {
    m_rad->SetDOFValues(toDblVec(x));
    VectorXd out = VectorXd::Zero(m_rad->GetDOF());
    BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
      if (!link->GetGeometries().empty()) {
        OR::Vector cm = link->GetGlobalCOM();
        DblMatrix jac = m_rad->PositionJacobian(link->GetIndex(), cm) * link->GetMass();
        out += jac.row(2).transpose();
      }
    }
    return out;
  }
};

StaticTorqueCost::StaticTorqueCost(RobotAndDOFPtr rad, const VarVector& vars, double coeff) :
  CostFromErrFunc(VectorOfVectorPtr(new StaticTorqueCostCalc(rad)), vars, VectorXd::Ones(vars.size())*coeff,
    SQUARED,  "static_torque") {
}

struct PECalc : public VectorOfVector {
  RobotAndDOFPtr m_rad;
  PECalc(const RobotAndDOFPtr rad) : m_rad(rad) {}
  VectorXd operator()(const VectorXd& x) const {
    m_rad->SetDOFValues(toDblVec(x));
    VectorXd out = VectorXd::Zero(1);
    BOOST_FOREACH(const KinBody::LinkPtr& link, m_rad->GetRobot()->GetLinks()) {
      if (!link->GetGeometries().empty()) {
        OR::Vector cm = link->GetGlobalCOM();
        out(0) += cm.z;
      }
    }
    return out;
  }
};

PECost::PECost(RobotAndDOFPtr rad, const VarVector& vars, double coeff) :
  CostFromErrFunc(VectorOfVectorPtr(new PECalc(rad)), vars, VectorXd::Ones(1)*coeff,
    SQUARED,  "PE") {
}

inline OpenRAVE::Vector toRaveVector(const Vector3d& x) {
  return OpenRAVE::Vector(x[0], x[1], x[2]);
}

struct FootHeightCalc : public VectorOfVector {
  RobotAndDOFPtr m_rad;
  KinBody::LinkPtr m_link;
  double m_height;
  FootHeightCalc(const RobotAndDOFPtr rad, KinBody::LinkPtr link, double height) : m_rad(rad), m_link(link), m_height(height) {}
  VectorXd operator()(const VectorXd& x) const {
    m_rad->SetDOFValues(toDblVec(x));
    OpenRAVE::Transform T = m_link->GetTransform();
    Eigen::Vector4d out(
        m_height - (T * toRaveVector(footbottompoints.row(0))).z,
        m_height - (T * toRaveVector(footbottompoints.row(1))).z,
        m_height - (T * toRaveVector(footbottompoints.row(2))).z,
        m_height - (T * toRaveVector(footbottompoints.row(3))).z);
    cout << out.transpose() << endl;
    cout << footbottompoints.row(0) << endl;
    return out;
  }
};

FootHeightConstraint::FootHeightConstraint(RobotAndDOFPtr rad, KinBody::LinkPtr link, double height, const VarVector& vars) :
  ConstraintFromFunc(VectorOfVectorPtr(new FootHeightCalc(rad, link, height)), vars, INEQ,  "FootHeight") {
}


extern ProblemConstructionInfo* gPCI;


struct PECostInfo : public CostInfo {
  double coeff;
  int timestep;
  void fromJson(const Value& v) {
    FAIL_IF_FALSE(v.isMember("params"));
    const Value& params = v["params"];
    childFromJson(params, timestep, "timestep");
    childFromJson(params, coeff, "coeff");
    int n_steps = gPCI->basic_info.n_steps;
    FAIL_IF_FALSE((timestep >= 0) && (timestep < n_steps));
  }
  void hatch(TrajOptProb& prob) {
    prob.addCost(CostPtr(new PECost(prob.GetRAD(), prob.GetVarRow(timestep), coeff)));
    prob.getCosts().back()->setName(name);
  }
  static CostInfoPtr create() {
    return CostInfoPtr(new PECostInfo());
  }
};
struct StaticTorqueCostCostInfo : public CostInfo {
  double coeff;
  int timestep;
  void fromJson(const Value& v) {
    FAIL_IF_FALSE(v.isMember("params"));
    const Value& params = v["params"];
    childFromJson(params, timestep, "timestep");
    childFromJson(params, coeff, "coeff");
    int n_steps = gPCI->basic_info.n_steps;
    FAIL_IF_FALSE((timestep >= 0) && (timestep < n_steps));
  }
  void hatch(TrajOptProb& prob) {
    prob.addCost(CostPtr(new StaticTorqueCost(prob.GetRAD(), prob.GetVarRow(timestep), coeff)));
    prob.getCosts().back()->setName(name);    
  }
  static CostInfoPtr create() {
    return CostInfoPtr(new StaticTorqueCostCostInfo());
  }
};

template <typename MatrixT>
MatrixT concat0(const MatrixT& x, const MatrixT& y) {
  MatrixT out(x.rows() + y.rows(), x.cols());
  out.topRows(x.rows()) = x;
  out.bottomRows(y.rows()) = y;
  return out;
}


MatrixX2d GetFootPoly(const KinBody::Link& link) {
  OpenRAVE::Vector v = link.GetTransform().trans;
  v.z = 0;
  return local_aabb_poly.rowwise() + Vector2d(v.x, v.y).transpose();
}
MatrixX2d GetFeetPoly(const vector<KinBody::LinkPtr>& links) {
  FAIL_IF_FALSE(links.size() > 0);
  MatrixX2d allpoly = GetFootPoly(*links[0]);
  for (int i=1; i < links.size(); ++i) {
    allpoly = concat0(allpoly, GetFootPoly(*links[i]));
  }
  return hull2d(allpoly);
}


struct ZMPConstraintCntInfo : public CntInfo {
  int timestep;
  vector<string> planted_link_names;
  void fromJson(const Value& v) {
    FAIL_IF_FALSE(v.isMember("params"));
    const Value& params = v["params"];
    childFromJson(params, timestep, "timestep");
    int n_steps = gPCI->basic_info.n_steps;
    FAIL_IF_FALSE((timestep >= 0) && (timestep < n_steps));
    childFromJson(params, planted_link_names, "planted_links");
  }
  void hatch(TrajOptProb& prob) {
    vector<KinBody::LinkPtr> planted_links;
    BOOST_FOREACH(const string& linkname, planted_link_names) {
      KinBody::LinkPtr link = prob.GetRAD()->GetRobot()->GetLink(linkname);
      if (!link) {
        PRINT_AND_THROW(boost::format("invalid link name: %s")%linkname);
      }
      planted_links.push_back(link);
    }
    prob.addConstr(ConstraintPtr(new ZMPConstraint(prob.GetRAD(), GetFeetPoly(planted_links), prob.GetVarRow(timestep))));
    prob.getIneqConstraints().back()->setName(name);    
  }
  static CntInfoPtr create() {
    return CntInfoPtr(new ZMPConstraintCntInfo());
  }
};

struct FootHeightCntInfo : public CntInfo {
  int timestep;
  double height;
  string link_name;
  void fromJson(const Value& v) {
    FAIL_IF_FALSE(v.isMember("params"));
    const Value& params = v["params"];
    childFromJson(params, timestep, "timestep");
    childFromJson(params, height, "height");
    int n_steps = gPCI->basic_info.n_steps;
    FAIL_IF_FALSE((timestep >= 0) && (timestep < n_steps));
    childFromJson(params, link_name, "link");
  }
  void hatch(TrajOptProb& prob) {
    KinBody::LinkPtr link = prob.GetRAD()->GetRobot()->GetLink(link_name);
    if (!link) {
      PRINT_AND_THROW(boost::format("invalid link name: %s")%link_name);
    }
    prob.addConstr(ConstraintPtr(new FootHeightConstraint(prob.GetRAD(), link, height, prob.GetVarRow(timestep))));
    prob.getIneqConstraints().back()->setName(name);    
  }
  static CntInfoPtr create() {
    return CntInfoPtr(new FootHeightCntInfo());
  }
};



TRAJOPT_API void RegisterHumanoidCostsAndCnts() {
  CostInfo::RegisterMaker("potential_energy", &PECostInfo::create);
  CostInfo::RegisterMaker("static_torque", &StaticTorqueCostCostInfo::create);
  CntInfo::RegisterMaker("zmp", &ZMPConstraintCntInfo::create);
  CntInfo::RegisterMaker("foot_height", &FootHeightCntInfo::create);
}


}
