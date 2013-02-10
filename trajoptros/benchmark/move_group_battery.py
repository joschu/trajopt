#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scenefile")
parser.add_argument("--right", action="store_true")
parser.add_argument("--planner_id", default = "", choices = [
    "",
    "SBLkConfigDefault",
    "LBKPIECEkConfigDefault",
    "RRTkConfigDefault",
    "RRTConnectkConfigDefault",
    "ESTkConfigDefault",
    "KPIECEkConfigDefault",
    "BKPIECEkConfigDefault",
    "RRTStarkConfigDefault"])
parser.add_argument("--pause_after_response",action="store_true")
parser.add_argument("--max_planning_time", type=int, default=10)
parser.add_argument("--planner",type=str,default="unnamedplanner")
parser.add_argument("--save",action="store_true")
args = parser.parse_args()

if "counter" in args.scenefile:
    ROS_JOINT_NAMES =  ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_motor_slider_joint', 'r_gripper_motor_screw_joint', 'r_gripper_l_finger_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint', 'l_gripper_l_finger_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_joint', 'torso_lift_motor_screw_joint']
    ROS_DEFAULT_JOINT_VALS =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.21, 0.0, 0.0, 0.0, -1.5, 0.0179976, -0.00523819, -1.4, -0.000417087, -0.744712, -0.00100021, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 1.0, 0.00568301, -1.4, 0.000799433, -0.743912, 0.000498893, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
else:
    ROS_JOINT_NAMES = ['br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_motor_slider_joint', 'r_gripper_motor_screw_joint', 'r_gripper_l_finger_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint', 'l_gripper_l_finger_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_joint', 'torso_lift_motor_screw_joint', 'fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint']
    ROS_DEFAULT_JOINT_VALS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.657967, 0.888673, -1.4311, -1.073419, -0.705232, -1.107079, 2.806742, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.848628, 0.7797, 1.396294, -0.828274, 0.687905, -1.518703, 0.394348, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


import subprocess, os
import cPickle
from time import time
import roslib
import sys
import rospy
import moveit_msgs.msg as mm
import moveit_msgs.srv as ms
import geometry_msgs.msg as gm
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku
from trajoptpy.check_traj import check_traj
import actionlib
import time
import numpy as np
import json
import openravepy as rave, numpy as np
from visualization_msgs.msg import *


def build_robot_state(joint_names=ROS_JOINT_NAMES, joint_values=ROS_DEFAULT_JOINT_VALS):
    start_state = mm.RobotState()
    start_state.joint_state.name = joint_names
    start_state.joint_state.position = joint_values
    start_state.multi_dof_joint_state.joint_names =  ['world_joint']
    start_state.multi_dof_joint_state.frame_ids = ['odom_combined']
    start_state.multi_dof_joint_state.child_frame_ids = ['base_footprint']
    base_pose = gm.Pose()
    base_pose.orientation.w = 1
    start_state.multi_dof_joint_state.poses = [ base_pose ]
    return start_state

INITIAL_ROBOT_STATE = build_robot_state()



def alter_robot_state(robot_state, joints, values):
    d = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))
    for joint_name, value in zip(joints, values):
        d[joint_name] = value
    names, positions = zip(*d.items())
    robot_state.joint_state.name = names
    robot_state.joint_state.position = positions
    return robot_state
    

def build_joint_request(jvals, arm, robot, initial_state=INITIAL_ROBOT_STATE, planner_id=''):
    m = mm.MotionPlanRequest()
    m.group_name = "%s_arm" % arm
    m.start_state = initial_state
    m.planner_id = planner_id
    c = mm.Constraints()
    joints = robot.GetJoints()
    joint_inds = robot.GetManipulator("%sarm"%arm).GetArmIndices()
    c.joint_constraints = [mm.JointConstraint(joint_name=joints[joint_inds[i]].GetName(), position = jvals[i])
                           for i in xrange(len(jvals))]
    jc = mm.JointConstraint()

    m.goal_constraints = [c]
    m.allowed_planning_time = rospy.Duration(args.max_planning_time)
    base_pose = initial_state.multi_dof_joint_state.poses[0]
    base_q = base_pose.orientation
    base_p = base_pose.position
    t = rave.matrixFromPose([base_q.w, base_q.x, base_q.y, base_q.z, base_p.x, base_p.y, base_p.z])
    robot.SetTransform(t)

    return m
    
def build_constraints_request(arm, constraints, initial_state=INITIAL_ROBOT_STATE, planner_id=''):
    m = MotionPlanRequest()
    m.group_name = "%s_arm" % arm
    m.start_state = initial_state
    m.planner_id = planner_id
    m.goal_constraints = [constraints]
    return m


def build_cart_request(pos, quat, arm, initial_state = INITIAL_ROBOT_STATE, planner_id=''):
    m = MotionPlanRequest()
    m.group_name = "%s_arm" % arm
    target_link = "%s_wrist_roll_link" % arm[0]
    m.start_state = initial_state
    m.planner_id = planner_id

    pc = PositionConstraint()
    pc.link_name = target_link
    pc.header.frame_id = 'odom_combined'
    pose = gm.Pose()
    pose.position = pos

    pc.constraint_region.primitive_poses = [pose]
    
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [.01]
    pc.constraint_region.primitives = [ sphere ]
    
    oc = OrientationConstraint()
    oc.link_name = target_link
    oc.header.frame_id = 'odom_combined'
    oc.orientation = quat
    c = Constraints()
    c.position_constraints = [ pc ]
    c.orientation_constraints = [ oc ]
    m.goal_constraints = [ c ]
    m.allowed_planning_time = rospy.Duration(args.max_planning_time)
    return m
    
def robot_state_from_pose_goal(xyz, xyzw, arm, robot, initial_state = build_robot_state()):
    manip = robot.GetManipulator(arm + "arm")
    joint_solutions = ku.ik_for_link(rave.matrixFromPose(np.r_[xyzw[3], xyzw[:3], xyz]),
                                     manip, "%s_wrist_roll_link"%arm[0], 1, True)

    if len(joint_solutions) == 0:
        raise Exception
    joints = robot.GetJoints()
    joint_inds = robot.GetManipulator("%sarm"%arm).GetArmIndices()
    joint_names = [joints[joint_inds[i]].GetName() for i in xrange(len(joint_solutions[0]))]
    joint_values = [joint_solutions[0][i] for i in xrange(len(joint_solutions[0]))]
    new_state = alter_robot_state(initial_state, joint_names, joint_values)
    return new_state

def test_plan_to_pose(xyz, xyzw, leftright, robot, initial_state = build_robot_state(), planner_id='', target_link="%s_gripper_tool_frame"):
    manip = robot.GetManipulator(leftright + "arm")
    
    base_pose = initial_state.multi_dof_joint_state.poses[0]
    base_q = base_pose.orientation
    base_p = base_pose.position
    t = rave.matrixFromPose([base_q.w, base_q.x, base_q.y, base_q.z, base_p.x, base_p.y, base_p.z])
    robot.SetTransform(t)

    joint_solutions = ku.ik_for_link(rave.matrixFromPose(np.r_[xyzw[3], xyzw[:3], xyz]), manip, target_link%leftright[0], 1, True)

    if len(joint_solutions) == 0:
        print "pose is not reachable"
        return None

    m = build_joint_request(joint_solutions[0], leftright, robot, initial_state, planner_id=planner_id)

    try:
        response = get_motion_plan(m)
        if args.pause_after_response:
            raw_input("press enter to continue")
    except rospy.ServiceException:
        return dict(returned = False)

    mpr = response.motion_plan_response
    if mpr.error_code.val != 1:
        print "Planner returned error code", mpr.error_code.val
        return dict(returned = True, error_code = mpr.error_code.val, safe=False)
    else:
        traj =  [list(jtp.positions) for jtp in mpr.trajectory.joint_trajectory.points]
        return dict(returned = True, safe = not check_traj(traj, manip, 100), traj = traj, planning_time = mpr.planning_time.to_sec(), error_code = mpr.error_code.val)
    
def update_rave_from_ros(robot, ros_values, ros_joint_names):
    inds_ros2rave = np.array([robot.GetJointIndex(name) for name in ros_joint_names])
    good_ros_inds = np.flatnonzero(inds_ros2rave != -1) # ros joints inds with matching rave joint
    rave_inds = inds_ros2rave[good_ros_inds] # openrave indices corresponding to those joints
    rave_values = [ros_values[i_ros] for i_ros in good_ros_inds]
    robot.SetJointValues(rave_values[:20],rave_inds[:20])
    robot.SetJointValues(rave_values[20:],rave_inds[20:])
    

if __name__ == "__main__":
    
    envfile = "/tmp/%s.xml"%os.path.basename(args.scenefile)
    subprocess.check_call("python scene2xml.py %s %s"%(args.scenefile, envfile), shell=True)
    
    
        
    rospy.init_node("move_group_battery")    
    env = rave.Environment()
    env.Load("robots/pr2-beta-static.zae")
    loadsuccess = env.Load(envfile)    
    assert loadsuccess
    get_motion_plan = rospy.ServiceProxy('plan_kinematic_path', ms.GetMotionPlan)    
    print "waiting for plan_kinematic_path"
    get_motion_plan.wait_for_service()
    print "ok"
    
    robot = env.GetRobots()[0]
    update_rave_from_ros(robot, ROS_DEFAULT_JOINT_VALS, ROS_JOINT_NAMES)
  
    xs, ys, zs = np.mgrid[.35:.85:.05, 0:.5:.05, .8:.9:.1]

    results = []
    for (x,y,z) in zip(xs.flat, ys.flat, zs.flat):
        result = test_plan_to_pose([x,y,z], [0,0,0,1], "right" if args.right else "left" , robot, planner_id=args.planner_id)
        if result is not None: results.append(result)

    success_count, fail_count, no_answer_count, timeouts= 0,0,0,0
    for result in results:
        if result["returned"] and result["error_code"] == 1:
            if result["safe"] : success_count += 1
            else: fail_count += 1
        else:
            no_answer_count += 1
    print "success count:", success_count
    print "fail count:", fail_count
    print "no answer count:", no_answer_count
    
    
    if args.save:
        if not os.path.exists("../pklresults"):
            os.mkdir("../pklresults")
        scene = os.path.basename(args.scenefile).split('.')[0]
        fname = "../pklresults/%s-%s.pkl"%(scene, args.planner)
        print "saving results to ", fname
        with open(fname, "w") as fh: cPickle.dump((scene,args.planner,results), fh, -1)
    

