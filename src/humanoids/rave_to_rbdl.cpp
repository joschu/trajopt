#include "rave_to_rbdl.hpp"
#include <map>
#include <boost/foreach.hpp>
#include "trajopt/utils.hpp"
#include "trajopt/rave_utils.hpp"
using namespace OpenRAVE;
using namespace Eigen;
using namespace trajopt;
using namespace std;
using namespace util;

namespace trajopt {

Matrix3d toMatrix3d(const OpenRAVE::TransformMatrix& tm) {
  Matrix3d out;
  for (int i=0; i < 3; ++i) for (int j=0; j < 3; ++j) out(i,j) = tm.rot(i,j);
  return out;
}

rbdmath::SpatialTransform toSpatialTransform(const OpenRAVE::Transform& T) {
  return rbdmath::SpatialTransform(toMatrix3d(T).transpose(), toVector3d(T.trans));
}

OpenRAVE::Transform toRave(const rbdmath::SpatialTransform& T) {
  OpenRAVE::TransformMatrix tm;
  tm.trans = toRave(T.r);
  for (int i=0; i < 3; ++i)
    for (int j=0; j < 3; ++j)
      tm.rot(i,j) = T.E(j,i);
  return OpenRAVE::Transform(tm);
}


boost::shared_ptr<rbd::Model> MakeRBDLModel(OpenRAVE::RobotBasePtr robot, bool floating_base, Link2ID& link2id) {
  boost::shared_ptr<rbd::Model> model(new rbd::Model());
  model->Init();
  model->gravity = Vector3d (0., 0, -9.81);


  KinBody::LinkPtr base_link = robot->GetJoints().empty() ? robot->GetLinks()[0] : robot->GetJoints()[0]->GetHierarchyParentLink();
  unsigned base_id = 0;
  if (floating_base) base_id = model->SetFloatingBaseBody(rbd::Body(base_link->GetMass(), toVector3d(base_link->GetLocalCOM()), toMatrix3d(base_link->GetLocalInertia())));

  printf("base id: %i\n",base_id);
  map<KinBody::LinkPtr, OpenRAVE::Transform> link2Tjl;

  link2id[base_link] = base_id;
  link2Tjl[base_link] = robot->GetTransform().inverse() * base_link->GetTransform(); // T_robot_base

  BOOST_FOREACH(const KinBody::JointPtr& joint, robot->GetJoints()) {



    KinBody::LinkPtr parent = joint->GetHierarchyParentLink(),
                    child = joint->GetHierarchyChildLink();
    KinBody::JointType rave_type = joint->GetType();
    rbd::JointType rbd_type = (rave_type == KinBody::JointRevolute) ? rbd::JointTypeRevolute : rbd::JointTypePrismatic;
    rbd::Joint rbd_joint(rbd_type, toVector3d(joint->GetInternalHierarchyAxis()));

    OpenRAVE::Transform T_l0_j = joint->GetInternalHierarchyLeftTransform(),
                        T_j_l1 = joint->GetInternalHierarchyRightTransform();

    Matrix3d T_j_m = toMatrix3d(T_j_l1 * /* T_l1_m */ child->GetLocalMassFrame());
    Matrix3d inertia_mm = toMatrix3d(child->GetLocalInertia());
    Matrix3d inertia_jj = T_j_m.inverse() * inertia_mm * T_j_m;

    rbd::Body body(child->GetMass(), toVector3d(T_j_l1 * child->GetLocalCOM()), inertia_jj);
    cout << "----------" << endl;
    cout <<  joint->GetName() << " " << rave_type << endl;
    
    KinBody::LinkPtr realParent = parent;
    while (link2id.find(realParent) == link2id.end()) {
      cout << "realParent" << realParent->GetName() << endl;
      vector<KinBody::LinkPtr> parentLinks;
      realParent->GetParentLinks(parentLinks);
      assert(parentLinks.size() == 1);
      realParent = parentLinks[0];
    }
    
    
    cout << "parent: " << parent->GetName() << " " << link2id[parent] << endl;
    link2id[child] = model->AddBody(link2id[realParent], toSpatialTransform(link2Tjl[realParent] * T_l0_j), rbd_joint, body, child->GetName());
    cout << "child: " << child->GetName() << link2id[child] << endl;
    link2Tjl[child] = T_j_l1;
    cout << "---------" << endl;
  }

  return model;


}

}
