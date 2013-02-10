#!/usr/bin/env python

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scenefile")
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--multi_init", action="store_true")
parser.add_argument("--discrete",action="store_true")
args = parser.parse_args()

import sys
sys.path.append("/home/joschu/ros/moveit/devel/lib/python2.7/dist-packages")
sys.path.append("/home/ibrahima/moveit/devel/lib/python2.7/dist-packages")


import subprocess, os
from time import time
import trajoptpy
import rospy
import json
import moveit_msgs.msg as mm
import openravepy as rave, numpy as np
import moveit_msgs.srv as ms
import trajoptpy.math_utils as mu
import trajectory_msgs.msg as tm
import geometry_msgs.msg as gm
from trajoptpy.check_traj import check_traj

FAILURE = -1
SUCCESS = 1

GROUPMAP = {
    "left_arm":"leftarm",
    "right_arm":"rightarm",    
}

def mirror_arm_joints(x):
    "mirror image of joints (r->l or l->r)"
    return [-x[0],x[1],-x[2],x[3],-x[4],x[5],-x[6]]
LEFT_POSTURES = [
    [-0.243379, 0.103374, -1.6, -2.27679, 3.02165, -2.03223, -1.6209], #chest fwd
    [-1.68199, -0.088593, -1.6, -2.08996, 3.04403, -0.41007, -1.39646],# side fwd
    [-0.0428341, -0.489164, -0.6, -1.40856, 2.32152, -0.669566, -2.13699],# face up
    [0.0397607, 1.18538, -0.8, -0.756239, -2.84594, -1.06418, -2.42207]# floor down
]
def get_left_postures(): return LEFT_POSTURES
def get_right_postures(): return [mirror_arm_joints(posture) for posture in LEFT_POSTURES]
def get_postures(leftright):
    if leftright=="left": return get_right_postures()
    if leftright=="right": return get_left_postures()
    raise Exception

def update_rave_from_ros(robot, ros_values, ros_joint_names):
    inds_ros2rave = np.array([robot.GetJointIndex(name) for name in ros_joint_names])
    good_ros_inds = np.flatnonzero(inds_ros2rave != -1) # ros joints inds with matching rave joint
    rave_inds = inds_ros2rave[good_ros_inds] # openrave indices corresponding to those joints
    rave_values = [ros_values[i_ros] for i_ros in good_ros_inds]
    robot.SetJointValues(rave_values[:20],rave_inds[:20])
    robot.SetJointValues(rave_values[20:],rave_inds[20:])   

def callback(getreq):
    assert isinstance(getreq, ms.GetMotionPlanRequest)
    req = getreq.motion_plan_request   
    getresp = ms.GetMotionPlanResponse()
    resp = getresp.motion_plan_response
    
    manip = GROUPMAP[req.group_name]
    
    update_rave_from_ros(robot, req.start_state.joint_state.position, req.start_state.joint_state.name)
    base_pose = req.start_state.multi_dof_joint_state.poses[0]
    base_q = base_pose.orientation
    base_p = base_pose.position
    t = rave.matrixFromPose([base_q.w, base_q.x, base_q.y, base_q.z, base_p.x, base_p.y, base_p.z])
    robot.SetTransform(t)
    start_joints = robot.GetDOFValues(robot.GetManipulator(manip).GetArmIndices())
    
    if args.discrete: n_steps = 18
    else: n_steps = 11
    coll_coeff = 10
    dist_pen = .04
    
    inds = robot.GetManipulator(manip).GetArmJoints()
    alljoints = robot.GetJoints()
    names = arm_joint_names = [alljoints[i].GetName() for i in inds]
        

    for goal_cnt in req.goal_constraints:
        if len(goal_cnt.joint_constraints) > 0:
            assert len(goal_cnt.joint_constraints)==7
            end_joints = np.zeros(7)
            for i in xrange(7):
                jc = goal_cnt.joint_constraints[i]
                dof_idx = arm_joint_names.index(jc.joint_name)
                end_joints[dof_idx] = jc.position

    waypoint_step = (n_steps - 1)// 2
    joint_waypoints = [(np.asarray(start_joints) + np.asarray(end_joints))/2]
    if args.multi_init:
        leftright = req.group_name.split("_")[0]
        joint_waypoints.extend(get_postures(leftright))
    
    success = False
    for waypoint in joint_waypoints:
        robot.SetDOFValues(start_joints, inds)
        d = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : manip,
                "start_fixed" : True
            },
            "costs" : [
            {
                "type" : "joint_vel",
                "params": {"coeffs" : [1]}
            },            
            {
                "type" : "collision" if args.discrete else "continuous_collision",
                "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen]}
            }
            ],
            "constraints" : [],
            "init_info" : {
                "type" : "given_traj"
            }
        }
        
        
        d["constraints"].append({
            "type" : "joint",
            "name" : "joint", 
            "params" : {
                "vals" : end_joints.tolist()
            }                
        })
        
        if args.multi_init:        
            print "trying with midpoint", waypoint
            inittraj = np.empty((n_steps, 7))
            inittraj[:waypoint_step+1] = mu.linspace2d(start_joints, waypoint, waypoint_step+1)
            inittraj[waypoint_step:] = mu.linspace2d(waypoint, end_joints, n_steps - waypoint_step)
        else:
            inittraj = mu.linspace2d(start_joints, end_joints, n_steps)
            
                
        d["init_info"]["data"] = [row.tolist() for row in inittraj]
    
                
        if len(goal_cnt.position_constraints) > 0:
            raise NotImplementedError
                
        
        s = json.dumps(d)
        t_start = time()
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
        planning_duration_seconds = time() - t_start
        
        traj = result.GetTraj()
        
        if check_traj(traj, robot.GetManipulator(manip)):                       
            print "aw, there's a collision"
        else:
            print "no collisions"
            success = True
            break
    if not success:
        resp.error_code.val = FAILURE
        return resp

    resp.trajectory.joint_trajectory.joint_names = names
    for row in traj:
        jtp = tm.JointTrajectoryPoint()
        jtp.positions = row.tolist()
        jtp.time_from_start = rospy.Duration(0)
        resp.trajectory.joint_trajectory.points.append(jtp)
        
    mdjt = resp.trajectory.multi_dof_joint_trajectory
    mdjt.joint_names =  ['world_joint']
    mdjt.frame_ids = ['odom_combined']
    mdjt.child_frame_ids = ['base_footprint']
    mdjt.header = req.start_state.joint_state.header
    pose = gm.Pose()
    pose.orientation.w = 1
    for _ in xrange(len(resp.trajectory.joint_trajectory.points)): 
        jtp = mm.MultiDOFJointTrajectoryPoint()
        jtp.poses.append(pose)
    #for i in xrange(len(mdjs.joint_names)):
        #if mdjs.joint_names[i] == "world_joint":
            #world_joint_pose = mdjs.poses[i]
            #world_joint_frame_id = mdjs.frame_ids[i]
            #world_joint_child_frame_id = mdjs.child_frame_ids[i]
    #print "world_joint_frame_id", world_joint_frame_id
    #print "world_joint_child_frame_id", world_joint_child_frame_id

    #mdjt = resp.trajectory.multi_dof_joint_trajectory
    #mdjt.header.frame_id = req.start_state.joint_state.header.frame_id
    #mdjt.joint_names.append("world_joint")

    resp.error_code.val = SUCCESS
    resp.planning_time = rospy.Duration(planning_duration_seconds)
    resp.trajectory.joint_trajectory.header = req.start_state.joint_state.header
    resp.group_name = req.group_name
    resp.trajectory_start.joint_state = req.start_state.joint_state
    
    #resp.trajectory.multi_dof_joint_trajectory
    #resp.trajectory.multi_dof_joint_trajectory.header = req.start_state.joint_state.header
    
    getresp.motion_plan_response = resp
    return getresp
if __name__ == "__main__":
    rospy.init_node("trajopt_planning_server")
    trajoptpy.SetInteractive(args.interactive)

    envfile = "/tmp/%s.xml"%os.path.basename(args.scenefile)
    subprocess.check_call("python scene2xml.py %s %s"%(args.scenefile, envfile), shell=True)
    env = rave.Environment()
    env.StopSimulation()
    env.Load("robots/pr2-beta-static.zae")
    env.Load(envfile)
    robot = env.GetRobots()[0]
    if args.interactive:
        viewer = trajoptpy.GetViewer(env)
        viewer.Step()
        viewer.SetAllTransparency(.6)
    
    svc = rospy.Service('/plan_kinematic_path', ms.GetMotionPlan, callback)
    rospy.spin()
