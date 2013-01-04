import openravepy
import jds_utils.conversions as conv

class IKFail(Exception):
    pass


def joint_traj_from_cart_traj(xyzs, quats, manip, link_name, filter_options = 0):
    "do ik and then fill in the points where ik failed"

    n = len(xyzs)
    assert len(quats) == n
    
    robot = manip.GetRobot()
    joint_inds = manip.GetArmJoints()
    robot.SetActiveDOFs(joint_inds)
    orig_joint = robot.GetActiveDOFValues()
    
    link = robot.GetLink(link_name)
    
    joints = []
    inds = []

    T_w_link = link.GetTransform()
    T_w_ee = robot.GetEndEffectorTransform()
    Tf_link_ee = T_w_link.inverse() * T_w_ee


    for i in xrange(0,n):
        T_w_link = conv.trans_rot_to_hmat(xyzs[i], quats[i])
        T_w_ee = T_w_link.dot(Tf_link_ee)

        joint = manip.FindIKSolution(T_w_ee, filter_options)
        if joint is not None: 
            joints.append(joint)
            inds.append(i)
            robot.SetActiveDOFValues(joint)

    robot.SetActiveDOFValues(orig_joint)
    
    if len(inds) > 2:
        joints2 = mu.interp2d(np.arange(n), inds, joints)
        return joints2, inds
    else:
        raise IKFail
    