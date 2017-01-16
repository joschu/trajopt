import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()


import openravepy, trajoptpy
from trajoptpy.check_traj import traj_is_safe
from trajoptpy.animate_traj import animate_traj
import numpy as np
import os.path as osp
import json


def make_fullbody_request(end_joints):
    if isinstance(end_joints, np.ndarray): end_joints = end_joints.tolist()
    n_steps = 30
    coll_coeff = 20
    dist_pen = .05
    d = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "active",
            "start_fixed" : True
        },
        "costs" : [
            {
                "type" : "joint_vel",
                "params": {"coeffs" : [1]}
            },            
            {
                "name" : "cont_coll",
                "type" : "collision",
                "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":True}
            },
            {
                "name": "disc_coll",
                "type" : "collision",
                "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":False}
            }            
        ],
        "constraints" : [
            {"type" : "joint", "params" : {"vals" : end_joints}}
        ],
        "init_info" : {
            "type" : "straight_line",
            "endpoint": end_joints
        }
    }
    
    return d


def main():
    env = openravepy.Environment()
    env.Load(osp.join(trajoptpy.data_dir, "kitchen.env.xml"))
    env.Load("robots/pr2-beta-static.zae")
    env.StopSimulation()
    robot = env.GetRobots()[0]


    # Set up robot in initial state
    robot.SetDOFValues([  0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,
                            0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,   0.000e+00,
                            0.000e+00,   0.000e+00,   0.0115e+00,   0.000e+00,   0.000e+00,
                            6.648e-01,  -3.526e-01,   1.676e+00,  -1.924e+00,   2.921e+00,
                            -1.217e+00,   1.343e+00,   0.000e+00,  -4.163e-16,   0.000e+00,
                            -1.665e-16,   0.000e+00,  -6.365e-01,   9.806e-02,  -1.226e+00,
                            -2.026e+00,  -3.012e+00,  -1.396e+00,  -1.929e+00,   0.000e+00,
                            2.776e-17,   0.000e+00,  -3.331e-16,   0.000e+00])
    robot.SetTransform(np.array([[-1.   ,  0.005,  0.   ,  2.93 ],
                                 [-0.005, -1.   ,  0.   ,  0.575],
                                 [ 0.   ,  0.   ,  1.   ,  0.   ],
                                 [ 0.   ,  0.   ,  0.   ,  1.   ]]))
    DOF = openravepy.DOFAffine                             
    robot.SetActiveDOFs(np.r_[robot.GetJoint("torso_lift_joint").GetDOFIndex(), robot.GetManipulator("leftarm").GetArmIndices(), robot.GetManipulator("rightarm").GetArmIndices()],
        DOF.X | DOF.Y | DOF.RotationAxis)
    robot.SetActiveDOFValues([ 0.115 ,  0.6648, -0.3526,  1.6763, -1.9242,  2.9209, -1.2166,
                            1.3425, -0.6365,  0.0981, -1.226 , -2.0264, -3.0125, -1.3958,
                            -1.9289,  2.9295,  0.5748, -3.137 ])
    target_joints = [ 0.115 ,  0.6808, -0.3535,  1.4343, -1.8516,  2.7542, -1.2005,
                    1.5994, -0.6929, -0.3338, -1.292 , -1.9048, -2.6915, -1.2908,
                   -1.7152,  1.3155,  0.6877, -0.0041]                            
    
    # Planner assumes current state of robot in OpenRAVE is the initial state
    request = make_fullbody_request(target_joints)
    s = json.dumps(request) # convert dictionary into json-formatted string
    prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
    result = trajoptpy.OptimizeProblem(prob) # do optimization
    
    traj = result.GetTraj()

    success = traj_is_safe(traj, robot)
    if success:
        print "trajectory is safe! :)"
    else:
        print "trajectory contains a collision :("
    
    if args.interactive: animate_traj(traj, robot)
    
    assert success
                                                

if __name__ == "__main__":
    main()