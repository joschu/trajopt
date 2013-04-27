import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()


import trajoptpy
import openravepy as rave
import numpy as np
import json
import atexit # avoid segfault  at exit
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku

def smooth_traj_request(robot, traj):
    n_dof = robot.GetActiveDOF()

    request = {
        "basic_info" : {
            "n_steps" : traj.GetNumWaypoints(),
            "manip" : "active",
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : [1]}
        },            
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.025]}
        }
        ],
        "constraints" : [
        {
            "type" : "joint",
            "params" : {
                "vals" : traj.GetWaypoint(traj.GetNumWaypoints()-1)[:n_dof].tolist()
            },
        }
        ],
        "init_info" : {
            "type": "given_traj",
            "data" : [traj.GetWaypoint(i)[:n_dof].tolist() for i in xrange(traj.GetNumWaypoints())]
        }
    }
    
    return request


if __name__ == "__main__":
        
    ### Parameters ###
    ENV_FILE = "../data/pr2_table.env.xml"
    N_STEPS = 15
    XYZ_TARGET = [.5,0,.9]
    QUAT_TARGET = [1,0,0,0]
    INTERACTIVE = True
    LINK_NAME = "r_gripper_tool_frame"
    ##################
    
    
    ### Env setup ####
    env = rave.RaveGetEnvironment(1)
    if env is None:
        env = rave.Environment()
        env.StopSimulation()
        atexit.register(rave.RaveDestroy)
        env.Load(ENV_FILE)
    robot = env.GetRobots()[0]
    robot.SetActiveDOFs(robot.GetManipulator("leftarm").GetArmJoints())
    robot.SetActiveDOFValues(np.zeros(robot.GetActiveDOF()))

    basemanip = rave.interfaces.BaseManipulation(robot)
    print "starting RRT planning..."
    traj=basemanip.MoveActiveJoints(goal=[0.609648, 1.37131, 1.6, -1.05298, -1.41295, -0.979627, 0.93925], 
                                    outputtrajobj=True,execute=False)    
    print "RRT done"
    if traj.GetNumWaypoints() == 0:
        raise Exception("planner couldn't find a path")
    ##################
    request = smooth_traj_request(robot, traj)
    s = json.dumps(request)
    print json.dumps(request, indent=1)
    trajoptpy.SetInteractive(args.interactive);
    prob = trajoptpy.ConstructProblem(s, env)
    result = trajoptpy.OptimizeProblem(prob)
