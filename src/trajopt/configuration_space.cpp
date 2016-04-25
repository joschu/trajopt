#include "trajopt/configuration_space.hpp"
#include <boost/foreach.hpp>
#include "trajopt/rave_utils.hpp"
#include "utils/math.hpp"
using namespace OpenRAVE;
using namespace util;

// TODO: configuration should know something about what dofs are part of SO(1), SO(2) etc for functions like RandomDOFValues

namespace trajopt {

void KinBodyAndDOF::SetDOFValues(const DblVec& dofs) {
  if (m_affinedofs != 0) {
    OR::Transform T = m_kinbody->GetTransform();
    OR::RaveGetTransformFromAffineDOFValues(T, dofs.begin(), m_affinedofs, m_rotationaxis, true);
    m_kinbody->SetTransform(T);
  }
}

DblVec KinBodyAndDOF::GetDOFValues() {
  DblVec out;
  if (m_affinedofs != 0) {
    out.resize(GetDOF());
    OR::Transform T = m_kinbody->GetTransform();
    OR::RaveGetAffineDOFValuesFromTransform(out.begin(), T, m_affinedofs, m_rotationaxis);
  }
  return out;
}

void KinBodyAndDOF::GetDOFLimits(DblVec& lower, DblVec& upper) const {
  const int translation_dofs[3] = {DOF_X, DOF_Y, DOF_Z};
  for (int i=0; i < 3; ++i) {
    if (m_affinedofs & translation_dofs[i]) {
      lower.push_back(-INFINITY);
      upper.push_back(INFINITY);
    }
  }
  if (m_affinedofs & DOF_RotationMask) {
    if (m_affinedofs & DOF_RotationAxis) {
      lower.push_back(-INFINITY);
      upper.push_back(INFINITY);
    }
    else if (m_affinedofs & DOF_Rotation3D) {
      for (int i=0; i < 3; ++i) {
        lower.push_back(-INFINITY);
        upper.push_back(INFINITY);
      }
    }
    else if (m_affinedofs & DOF_RotationQuat) {
      for (int i=0; i < 4; ++i) {
        lower.push_back(-1);
        upper.push_back(1);
      }
    }
    else throw OR::openrave_exception("invalid rotation dofs", ORE_InvalidArguments);
  }
}

int KinBodyAndDOF::GetDOF() const {
  return RaveGetAffineDOF(m_affinedofs);
}

vector<KinBodyPtr> KinBodyAndDOF::GetBodies() {
  std::set<KinBodyPtr> bodies;
  m_kinbody->GetAttached(bodies);
  return vector<KinBodyPtr> (bodies.begin(), bodies.end());
}

bool KinBodyAndDOF::DoesAffect(const KinBody::Link& link) {
  return (m_affinedofs > 0) ? true : false; 
}

std::vector<KinBody::LinkPtr> KinBodyAndDOF::GetAffectedLinks() {
  vector<int> inds;
  std::vector<KinBody::LinkPtr> out;
  GetAffectedLinks(out, false, inds);
  return out;
}

void KinBodyAndDOF::GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) {
  links.clear();
  link_inds.clear();
  BOOST_FOREACH(const KinBody::LinkPtr& link, m_kinbody->GetLinks()) {
    if (this->DoesAffect(*link) && !(only_with_geom && link->GetGeometries().empty())) {
      links.push_back(link);
      link_inds.push_back(link->GetIndex());
    }
  }
}

DblMatrix KinBodyAndDOF::PositionJacobian(int link_ind, const OR::Vector& pt) const {
  Configuration::SaverPtr saver = const_cast<KinBodyAndDOF*>(this)->Save();

  /****************************************************************
  *    Title: OpenRAVE
  *    Author: Rosen Diankov
  *    Date: March 18, 2013
  *    Code version: 0.9.0
  *    Availability: https://github.com/rdiankov
  *****************************************************************/

  vector<double> jacdata;
  int num_dofs = GetDOF();
  jacdata.resize(3*num_dofs);
  
  if (m_affinedofs == DOF_NoTransform) { return DblMatrix(); }
 
  size_t ind = 0;
  if (m_affinedofs & DOF_X) {
      jacdata[ind] = 1;
      jacdata[num_dofs+ind] = 0;
      jacdata[2*num_dofs+ind] = 0;
      ind++;
  }
  if (m_affinedofs & DOF_Y) {
      jacdata[ind] = 0;
      jacdata[num_dofs+ind] = 1;
      jacdata[2*num_dofs+ind] = 0;
      ind++;
  }
  if (m_affinedofs & DOF_Z) {
      jacdata[ind] = 0;
      jacdata[num_dofs+ind] = 0;
      jacdata[2*num_dofs+ind] = 1;
      ind++;
  }
  if (m_affinedofs & DOF_RotationAxis) {
      Vector rotj = m_rotationaxis.cross(pt - m_kinbody->GetTransform().trans);
      jacdata[ind] = rotj.x;
      jacdata[num_dofs+ind] = rotj.y;
      jacdata[2*num_dofs+ind] = rotj.z;
      ind++;
  }
  else if (m_affinedofs & DOF_Rotation3D) {
      // have to take the partial derivative dT/dA of the axis*angle representation with respect to the transformation it induces
      // can introduce converting to quaternions in the middle, then by chain rule,  dT/dA = dT/tQ * dQ/dA
      // for questions on derivation email rdiankov@cs.cmu.edu
      Transform t = m_kinbody->GetTransform();
      double Qx = t.rot.x, Qy = t.rot.y, Qz = t.rot.z, Qw = t.rot.w;
      double Tx = pt.x-t.trans.x, Ty = pt.y-t.trans.y, Tz = pt.z-t.trans.z;
      // after some math, the dT/dQ looks like:
      double dRQ[12] = { 2*Qy*Ty+2*Qz*Tz,         -4*Qy*Tx+2*Qx*Ty+2*Qw*Tz,   -4*Qz*Tx-2*Qw*Ty+2*Qx*Tz,   -2*Qz*Ty+2*Qy*Tz,
                        2*Qy*Tx-4*Qx*Ty-2*Qw*Tz, 2*Qx*Tx+2*Qz*Tz,            2*Qw*Tx-4*Qz*Ty+2*Qy*Tz,    2*Qz*Tx-2*Qx*Tz,
                        2*Qz*Tx+2*Qw*Ty-4*Qx*Tz, -2*Qw*Tx+2*Qz*Ty-4*Qy*Tz,   2*Qx*Tx+2*Qy*Ty,            -2*Qy*Tx+2*Qx*Ty };
      // calc dQ/dA
      double fsin = sqrt(t.rot.y * t.rot.y + t.rot.z * t.rot.z + t.rot.w * t.rot.w);
      double fcos = t.rot.x;
      double fangle = 2 * atan2(fsin, fcos);
      double normalizer = fangle / fsin;
      double Ax = normalizer * t.rot.y;
      double Ay = normalizer * t.rot.z;
      double Az = normalizer * t.rot.w;
      if( RaveFabs(fangle) < 1e-8f )
          fangle = 1e-8f;
      double fangle2 = fangle*fangle;
      double fiangle2 = 1/fangle2;
      double inormalizer = normalizer > 0 ? 1/normalizer : 0;
      double fconst = inormalizer*fiangle2;
      double fconst2 = fcos*fiangle2;
      double dQA[12] = {-0.5f*Ax*inormalizer,                     -0.5f*Ay*inormalizer,                       -0.5f*Az*inormalizer,
                        inormalizer+0.5f*Ax*Ax*(fconst2-fconst),  0.5f*Ax*fconst2*Ay-Ax*fconst*Ay,            0.5f*Ax*fconst2*Az-Ax*fconst*Az,
                        0.5f*Ax*fconst2*Ay-Ax*fconst*Ay,          inormalizer+0.5f*Ay*Ay*(fconst2-fconst),    0.5f*Ay*fconst2*Az-Ay*fconst*Az,
                        0.5f*Ax*fconst2*Az-Ax*fconst*Az,          0.5f*Ay*fconst2*Az-Ay*fconst*Az,            inormalizer+0.5f*Az*Az*(fconst2-fconst)};
      for(int i = 0; i < 3; ++i) {
          for(int j = 0; j < 3; ++j) {
              jacdata[i*num_dofs+ind+j] = dRQ[4*i+0]*dQA[3*0+j] + dRQ[4*i+1]*dQA[3*1+j] + dRQ[4*i+2]*dQA[3*2+j] + dRQ[4*i+3]*dQA[3*3+j];
          }
      }
      ind += 3;
  }
  else if (m_affinedofs & DOF_RotationQuat) {
      Transform t = m_kinbody->GetTransform();
      // note: qw, qx, qy, qz here follow the standard quaternion convention, not the openrave one
      double qw = t.rot[0], qx = t.rot[1], qy = t.rot[2], qz = t.rot[3];
      Vector pt_local = t.inverse() * pt;
      double x = pt_local.x, y = pt_local.y, z = pt_local.z;
      double dRQ[12] = {-2*qz*y + 2*qw*x + 2*qy*z,2*qx*x + 2*qy*y + 2*qz*z,-2*qy*x + 2*qw*z + 2*qx*y,-2*qw*y - 2*qz*x + 2*qx*z,-2*qx*z + 2*qw*y + 2*qz*x,-2*qw*z - 2*qx*y + 2*qy*x,2*qx*x + 2*qy*y + 2*qz*z,-2*qz*y + 2*qw*x + 2*qy*z,-2*qy*x + 2*qw*z + 2*qx*y,-2*qx*z + 2*qw*y + 2*qz*x,-2*qw*x - 2*qy*z + 2*qz*y,2*qx*x + 2*qy*y + 2*qz*z};
      for (int i=0; i < 3; ++i) {
          double qdotrow = dRQ[4*i]*qw + dRQ[4*i+1]*qx + dRQ[4*i+2]*qy + dRQ[4*i+3]*qz;
          dRQ[4*i] -= qdotrow*qw;
          dRQ[4*i+1] -= qdotrow*qx;
          dRQ[4*i+2] -= qdotrow*qy;
          dRQ[4*i+3] -= qdotrow*qz;
      }
      for(int i = 0; i < 3; ++i) {
          for(int j = 0; j < 4; ++j) {
              jacdata[i*num_dofs+ind + j] = dRQ[4*i+j];
          }
      }
      ind += 4;
  }

  return Eigen::Map<DblMatrix>(jacdata.data(), 3, num_dofs);
}

void RobotAndDOF::SetDOFValues(const DblVec& dofs) {
  if (m_joint_inds.size() > 0)
    m_kinbody->SetDOFValues(dofs, false, m_joint_inds);
  if (m_affinedofs != 0) {
    OR::Transform T = m_kinbody->GetTransform();
    OR::RaveGetTransformFromAffineDOFValues(T, dofs.begin() + m_joint_inds.size(), m_affinedofs, m_rotationaxis);
    m_kinbody->SetTransform(T);
  }
}

DblVec RobotAndDOF::GetDOFValues() {
  DblVec out;
  if (m_joint_inds.size() > 0)
    m_kinbody->GetDOFValues(out, m_joint_inds);
  if (m_affinedofs != 0) {
    out.resize(GetDOF());
    OR::Transform T = m_kinbody->GetTransform();
    OR::RaveGetAffineDOFValuesFromTransform(out.begin() + m_joint_inds.size(), T, m_affinedofs, m_rotationaxis);
  }
  return out;
}

void RobotAndDOF::SetRobotActiveDOFs()  {
  RobotBasePtr robot = boost::dynamic_pointer_cast<RobotBase>(m_kinbody); // since robot is just a kinbody
  vector<int> current_active = robot->GetActiveDOFIndices();
  if (robot->GetActiveDOF() != GetDOF() || !std::equal(current_active.begin(), current_active.end(), m_joint_inds.begin()))
    robot->SetActiveDOFs(m_joint_inds, m_affinedofs);
}

void RobotAndDOF::GetDOFLimits(DblVec& lower, DblVec& upper) const {
  if (m_joint_inds.size() > 0)
    m_kinbody->GetDOFLimits(lower, upper, m_joint_inds);
  KinBodyAndDOF::GetDOFLimits(lower, upper);
}
int RobotAndDOF::GetDOF() const {
  return m_joint_inds.size() + RaveGetAffineDOF(m_affinedofs);
}
DblMatrix RobotAndDOF::PositionJacobian(int link_ind, const OR::Vector& pt) const {
  Configuration::SaverPtr saver = const_cast<RobotAndDOF*>(this)->Save();
  const_cast<RobotAndDOF*>(this)->SetRobotActiveDOFs();
  vector<double> jacdata;
  boost::dynamic_pointer_cast<RobotBase>(m_kinbody)->CalculateActiveJacobian(link_ind, pt, jacdata);
  return Eigen::Map<DblMatrix>(jacdata.data(), 3, GetDOF());
}
DblMatrix RobotAndDOF::RotationJacobian(int link_ind, const OR::Vector& rot) const {
  Configuration::SaverPtr saver = const_cast<RobotAndDOF*>(this)->Save();
  const_cast<RobotAndDOF*>(this)->SetRobotActiveDOFs();
  vector<double> jacdata;
  boost::dynamic_pointer_cast<RobotBase>(m_kinbody)->CalculateActiveRotationJacobian(link_ind, rot, jacdata);
  return Eigen::Map<DblMatrix>(jacdata.data(), 3, GetDOF());  
}
bool RobotAndDOF::DoesAffect(const KinBody::Link& link) {
  if (m_affinedofs > 0) return true;
  else if (link.GetParent() == m_kinbody) return trajopt::DoesAffect(*GetRobot(), m_joint_inds, GetRobotLinkIndex(*GetRobot(), link));
  else return false;
}

void RobotAndDOF::GetAffectedLinks(std::vector<KinBody::LinkPtr>& links, bool only_with_geom, vector<int>& link_inds) {
  KinBodyAndDOF::GetAffectedLinks(links, only_with_geom, link_inds);

  vector<KinBodyPtr> grabbed;
  boost::dynamic_pointer_cast<RobotBase>(m_kinbody)->GetGrabbed(grabbed);
  BOOST_FOREACH(const KinBodyPtr& body, grabbed) {
    KinBody::LinkPtr grabberLink = boost::dynamic_pointer_cast<RobotBase>(m_kinbody)->IsGrabbing(body);
    assert(grabberLink);
    BOOST_FOREACH(const KinBody::LinkPtr& link, body->GetLinks()) {
      if (link->GetGeometries().size() > 0) {
        links.push_back(link);
        link_inds.push_back(grabberLink->GetIndex());
      }
    }
  }

}

DblVec RobotAndDOF::RandomDOFValues() {
  int ndof = GetDOF();
  DblVec lower, upper;
  GetDOFLimits(lower, upper);
  DblVec out(ndof);
  for (int i=0; i < ndof; ++i) {
    // TODO this is only right for a circular joint!!
    lower[i] = fmax(lower[i], -2*M_PI);
    upper[i] = fmin(upper[i], 2*M_PI);
    out[i] = lower[i] + randf() * (upper[i] - lower[i]);
  }
  return out;
}

}
