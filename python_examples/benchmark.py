import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scenefile",choices=["tabletop", "bookshelves", "kitchen_counter"] )
parser.add_argument("--multi_init",type=bool, default=True)
parser.add_argument("--pose_goal", action="store_true")
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()


import trajoptpy, openravepy
from trajoptpy.check_traj import traj_is_safe
import trajoptpy.math_utils as mu
from time import time
import json
import numpy as np
import trajoptpy.kin_utils as ku
from collections import namedtuple
import os.path as osp

PlanResult = namedtuple("PlanResult", "success t_total t_opt t_verify")

def mirror_arm_joints(x):
    "mirror image of joints (r->l or l->r)"
    return [-x[0],x[1],-x[2],x[3],-x[4],x[5],-x[6]]
LEFT_POSTURES = [
    [-0.243379, 0.103374, -1.6, -2.27679, 3.02165, -2.03223, -1.6209], #chest fwd
    [-1.68199, -0.088593, -1.6, -2.08996, 3.04403, -0.41007, -1.39646],# side fwd
    [-0.0428341, -0.489164, -0.6, -1.40856, 2.32152, -0.669566, -2.13699],# face up
    [0.0397607, 1.18538, -0.8, -0.756239, -2.84594, -1.06418, -2.42207]# floor down
]
def get_postures(manip):
    if manip=="leftarm": return LEFT_POSTURES
    if manip=="rightarm": return [mirror_arm_joints(posture) for posture in LEFT_POSTURES]
    raise Exception



if args.scenefile == "tabletop": 
    dof_vals = \
        [ 0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,
        0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.849,
        0.78 ,  1.396, -0.828,  0.688, -1.519,  0.394,  0.   ,  0.   ,
        0.   ,  0.   ,  0.   , -0.658,  0.889, -1.431, -1.073, -0.705,
       -1.107,  2.807,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ]
    manip = "leftarm"   
elif args.scenefile == "bookshelves": 
    dof_vals = \
        [ 0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,
        0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ,  0.849,
        0.78 ,  1.396, -0.828,  0.688, -1.519,  0.394,  0.   ,  0.   ,
        0.   ,  0.   ,  0.   , -0.658,  0.889, -1.431, -1.073, -0.705,
       -1.107,  2.807,  0.   ,  0.   ,  0.   ,  0.   ,  0.   ]
    manip = "leftarm"   
elif args.scenefile == "kitchen_counter": 
    dof_vals = \
        [  0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,
         0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,
         0.000e+00,   0.000e+00,   2.100e-01,   0.000e+00,   0.000e+00,
         1.500e+00,   1.000e+00,   5.683e-03,  -1.400e+00,   7.994e-04,
        -7.439e-01,   4.989e-04,   0.000e+00,   0.000e+00,   0.000e+00,
         0.000e+00,   0.000e+00,  -1.500e+00,   1.800e-02,  -5.238e-03,
        -1.400e+00,  -4.171e-04,  -7.447e-01,  -1.000e-03,   0.000e+00,
         0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00]
    manip = "rightarm"   

def plan(robot, manip, end_joints, end_pose = None):
    
    arm_inds = robot.GetManipulator(manip).GetArmIndices()
    start_joints = robot.GetDOFValues(arm_inds)

    n_steps = 11
    coll_coeff = 10
    dist_pen = .05
    
    waypoint_step = (n_steps - 1)// 2
    joint_waypoints = [(np.asarray(start_joints) + np.asarray(end_joints))/2]
    joint_waypoints.extend(get_postures(manip))
    
    success = False
    t_start = time()
    t_verify = 0
    t_opt = 0
    
    for (i_init,waypoint) in enumerate(joint_waypoints):
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
                "type" : "collision",
                "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen]}
            },
            {
                "type" : "collision",
                "params" : {"continuous": False, "coeffs" : [coll_coeff],"dist_pen" : [dist_pen]}
            }
            ],
            "constraints" : [
                {"type" : "joint", "params" : {"vals" : end_joints.tolist()}} if end_pose is None else
                {"type" : "pose", "params" : {"xyz" : end_pose[4:7].tolist(), "wxyz" : end_pose[0:4].tolist(), "link":"%s_gripper_tool_frame"%manip[0]}}
            ],
            "init_info" : {
                "type" : "given_traj"
            }
        }
        
        if args.multi_init:        
            # print "trying with midpoint", waypoint
            inittraj = np.empty((n_steps, 7))
            inittraj[:waypoint_step+1] = mu.linspace2d(start_joints, waypoint, waypoint_step+1)
            inittraj[waypoint_step:] = mu.linspace2d(waypoint, end_joints, n_steps - waypoint_step)
        else:
            inittraj = mu.linspace2d(start_joints, end_joints, n_steps)                            
        d["init_info"]["data"] = [row.tolist() for row in inittraj]
                        
        s = json.dumps(d)
        prob = trajoptpy.ConstructProblem(s, env)
        
        t_start_opt = time()
        result = trajoptpy.OptimizeProblem(prob)
        t_opt += time() - t_start_opt
                
        traj = result.GetTraj()
        
        prob.SetRobotActiveDOFs()
        t_start_verify = time()
        is_safe = traj_is_safe(traj, robot)
        t_verify += time() - t_start_verify
        
        if not is_safe:                       
            print "optimal trajectory has a collision. trying a new initialization"
        else:
            print "planning successful after %s initialization"%(i_init+1)
            success = True
            break
    t_total = time() - t_start
    return PlanResult(success, t_total, t_opt, t_verify)        
        
trajoptpy.SetInteractive(args.interactive)
env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
loadsuccess = env.Load(osp.join(trajoptpy.data_dir, args.scenefile+".env.xml"))
assert loadsuccess
robot = env.GetRobots()[0]
robot.SetDOFValues(dof_vals)


# import xml.etree.ElementTree as ET
# cc = trajoptpy.GetCollisionChecker(env)
# root = ET.parse("/opt/ros/groovy/share/pr2_moveit_config/config/pr2.srdf").getroot()
# disabled_elems=root.findall("disable_collisions")
# for elem in disabled_elems:
#     linki = robot.GetLink(elem.get("link1"))
#     linkj = robot.GetLink(elem.get("link2"))
#     if linki and linkj:
#          cc.ExcludeCollisionPair(linki, linkj)



xs, ys, zs = np.mgrid[.35:.85:.05, 0:.5:.05, .8:.9:.1]
wxyz = [1,0,0,0]
results = []
for (x,y,z) in zip(xs.flat, ys.flat, zs.flat):
    wxyzxyz = np.r_[wxyz, x,y,z]
    joint_solutions = ku.ik_for_link(openravepy.matrixFromPose(wxyzxyz), robot.GetManipulator(manip), "%s_gripper_tool_frame"%manip[0], 1, True)
    if len(joint_solutions) > 0:
        target_joints = joint_solutions[0]
        results.append( plan(robot, manip, target_joints, wxyzxyz if args.pose_goal else None) )

success_frac = np.mean([result.success for result in results])
print "success frac:", success_frac
print "avg total time:", np.mean([result.t_total for result in results])
print "avg optimization time:", np.mean([result.t_opt for result in results])
print "avg verification time:", np.mean([result.t_verify for result in results])


  