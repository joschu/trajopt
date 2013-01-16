from brett2 import trajectories
from brett2.PR2 import PR2,mirror_arm_joints
import rospy
import numpy as np
from jds_utils.math_utils import linspace2d


if rospy.get_name() == "/unnamed":
    rospy.init_node("test_follow_rave_traj", anonymous=True)
    
pr2 = PR2.create()
pr2.update_rave()

dof_inds = []
dof_inds.extend(pr2.robot.GetManipulator("rightarm").GetArmIndices())
dof_inds.extend([pr2.robot.GetJoint("r_gripper_l_finger_joint").GetDOFIndex()])

n_steps = 20
ravetraj = np.zeros((n_steps, len(dof_inds)+3))

cur_arm_angs = pr2.rarm.get_joint_positions()
_,targ_angles = np.unwrap([cur_arm_angs, mirror_arm_joints(pr2.rarm.L_POSTURES["untucked"])], axis=0)
ravetraj[:,:7] = linspace2d(pr2.rarm.get_joint_positions(), targ_angles, n_steps)
ravetraj[:,7] = np.linspace(pr2.rgrip.get_angle(), .4*np.random.randn(), n_steps)
ravetraj[:,-3] = np.linspace(0,-.5, n_steps)

print "ravetraj", ravetraj


#part2traj = {}
#part2traj["base"] = ravetraj[:,-3:]
#trajectories.follow_body_traj2(pr2, part2traj,base_frame="base_footprint")
trajectories.follow_rave_trajectory(pr2, ravetraj, dof_inds, use_base = True,base_frame="/base_link")