import trajoptpy
import openravepy as rave
import numpy as np
import json
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku

def move_arm_to_grasp(xyz_targ, quat_targ, link_name, manip_name):
    
    request = {
        "basic_info" : {
            "n_steps" : 10,
            "manip" : manip_name,
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.025]}
        },
        {
            "type" : "joint_vel",
            "params": {"coeffs" : [1]}
        }        ],
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
                "first_step" : 7,
                "last_step" : 9, #inclusive
                "link" : link_name
            },
        }        
        
        ],
        "init_info" : {
            "type" : "stationary"
        }
    }
    
    return request


if __name__ == "__main__":
        
    ### Parameters ###
    ENV_FILE = "data/wamtest1.env.xml"
    MANIP_NAME = "arm"
    LINK_NAME = "wam7"
    INTERACTIVE = True
    ##################
    
    
    ### Env setup ####
    env = rave.RaveGetEnvironment(1)
    if env is None:
        env = rave.Environment()
        env.StopSimulation()
        env.Load(ENV_FILE)
    robot = env.GetRobots()[0]
    manip = robot.GetManipulator(MANIP_NAME)
    ##################
    
    T_w_mug = env.GetKinBody("mug-table-cluttered").GetLinks()[0].GetTransform()
    xyz_targ = (T_w_mug[:3,3] + np.array([0,0,.15])).tolist()
    quat_targ = np.array([0,1,0,0]).tolist()
    request = move_arm_to_grasp(xyz_targ, quat_targ, LINK_NAME, MANIP_NAME)
    s = json.dumps(request)
    print "REQUEST:",s
    trajoptpy.SetInteractive(INTERACTIVE);
    prob = trajoptpy.ConstructProblem(s, env)
    result = trajoptpy.OptimizeProblem(prob)