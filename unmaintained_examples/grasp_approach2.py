import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import trajoptpy
import openravepy as rave
import numpy as np
import json
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku
import trajoptpy.make_kinbodies as mk

def move_arm_to_grasp(xyz_targ, quat_targ, link_name, manip_name):
    
    request = {
        "basic_info" : {
            "n_steps" : 20,
            "manip" : manip_name,
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "collision",
            "params" : {"coeffs" : [1],"dist_pen" : [0.01]}
        },
        {
            "type" : "joint_vel",
            "params": {"coeffs" : [1]}
        },
        {
            "type" : "pose",
            "name" : "final_pose",
            "params" : {
                "pos_coeffs" : [100,100,100],
                "rot_coeffs" : [100,100,100],
                "xyz" : list(xyz_targ),
                "wxyz" : list(quat_targ),
                "link" : link_name,
            },
        }        
        ],
        "constraints" : [
        {
            "type" : "pose",
            "name" : "final_pose",
            "params" : {
                "pos_coeffs" : [1,1,1],
                "rot_coeffs" : [1,1,1],
                "xyz" : list(xyz_targ),
                "wxyz" : list(quat_targ),
                "link" : link_name,
            },
        },
        
        
        {
            "type" : "cart_vel",
            "name" : "cart_vel",
            "params" : {
                "distance_limit" : .01,
                "first_step" : 0,
                "last_step" : 19, #inclusive
                "link" : link_name
            },
        },        
        
        ],
        "init_info" : {
            "type" : "stationary"
        }
    }
    
    return request



if __name__ == "__main__":
        
    ### Parameters ###
    ENV_FILE = "robots/pr2-beta-static.zae"
    MANIP_NAME = "leftarm"
    LINK_NAME = "l_gripper_tool_frame"
     #0.0640945 0.704273 -0.147528 0.691468 -0.168806 -0.0452678 0.762821
    ##################
    
    ### Env setup ####
    env = rave.Environment()
    env.StopSimulation()
    env.Load(ENV_FILE)
    robot = env.GetRobots()[0]
    robot.SetDOFValues([ 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,-0.119839 ,0.861339 ,-2.22045e-16 ,-1.87834 ,-3.00095 ,-1.02143 ,3.0678 ,0.54 ,-1.01863e-14 ,0 ,-1.59595e-16 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,-8.16014e-15 ,0 ,-1.11022e-16 ,0])    
    manip = robot.GetManipulator(MANIP_NAME)
    ##################
    T_gripper = robot.GetLink(LINK_NAME).GetTransform()
    T_grasp = T_gripper.copy()
    T_grasp[:3,3]  += np.array([0,0.1,0])
    xyz_targ = T_grasp[:3,3]
    success = mk.create_cylinder(env, xyz_targ-np.array([.03,0,0]), .02, .5)
    quat_targ = rave.quatFromRotationMatrix(T_grasp[:3,:3])

    request = move_arm_to_grasp(xyz_targ, quat_targ, LINK_NAME, MANIP_NAME)
    s = json.dumps(request)
    print "REQUEST:",s
    trajoptpy.SetInteractive(args.interactive);
    prob = trajoptpy.ConstructProblem(s, env)
    result = trajoptpy.OptimizeProblem(prob)