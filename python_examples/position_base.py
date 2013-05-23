import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()


import trajoptpy
import openravepy as rave
import numpy as np
import json
from trajoptpy.check_traj import traj_is_safe


if rave.__version__ < "0.9":
    raise Exception("this example only works with openrave >= 0.9 due to recent jacobian bugfix")


def position_base_request(robot, link_name, xyz_targ, quat_targ):
        
    request = {
        # BEGIN basic_info
        "basic_info" : {
            "n_steps" : 1,
            "manip" : "active",
            "start_fixed" : False
        },
        # END basic_info
        "costs" : [
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.025]}
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
        }
        ],
        "init_info" : {
        }
    }

    # BEGIN random_init
    # randomly select joint values with uniform distribution over joint limits
    lower,upper = robot.GetActiveDOFLimits()
    lower = np.clip(lower, -np.pi, np.pi) # continuous joints have huge ranges, so we clip them to [-pi, pi]
    upper = np.clip(upper, -np.pi, np.pi) # to avoid poor numerical conditioning
    rands = np.random.rand(len(lower))
    dofvals_init = lower*rands + upper*(1-rands)
    # we'll treat the base pose specially, choosing a random angle and then setting a reasonable
    # position based on this angle
    angle_init = np.random.rand() * 2*np.pi
    x_init = xyz_targ[0] - .5*np.cos(angle_init)
    y_init = xyz_targ[1] - .5*np.sin(angle_init)    
    dofvals_init[-3:] = [x_init, y_init, angle_init]
    # END random_init

    request["init_info"]["type"] = "given_traj"
    request["init_info"]["data"] = [dofvals_init.tolist()]
    
    return request

def check_result(result, robot):
    print "checking trajectory for safety and constraint satisfaction..."
    success = True    
    if not traj_is_safe(result.GetTraj(), robot):
        success = False
        print "trajectory has a collision!"
    abstol = 1e-3
    for (name, val) in result.GetConstraints():
        if (val > abstol):
            success = False
            print "constraint %s wasn't satisfied (%.2e > %.2e)"%(name, val, abstol)
    return success
        
    

        
### Parameters ###
ENV_FILE = "data/pr2test1.env.xml"
XYZ_TARGET = [.5,0,.9]
QUAT_TARGET = [1,0,0,0]
LINK_NAME = "r_gripper_tool_frame"
##################
    
    
### Env setup ####
env = rave.Environment()
env.StopSimulation()
env.Load(ENV_FILE)
robot = env.GetRobots()[0]
robot.SetDOFValues([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
robot.SetTransform(rave.matrixFromPose([1, 0, 0, 0, -3.4, -1.4, 0.05]))

# BEGIN set_active
robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices(), 
                          robot.GetJoint("torso_lift_joint").GetDOFIndex()], 
                    rave.DOFAffine.X + rave.DOFAffine.Y + rave.DOFAffine.RotationAxis, [0,0,1])
# END set_active
##################
    
success = False
    
for i_try in xrange(100):
    request = position_base_request(robot, LINK_NAME, XYZ_TARGET, QUAT_TARGET)
    s = json.dumps(request)
    trajoptpy.SetInteractive(args.interactive)
    prob = trajoptpy.ConstructProblem(s, env)
    result = trajoptpy.OptimizeProblem(prob)
    if check_result(result, robot): 
        success = True
        break
        
if success:
    print "succeeded on try %i"%(i_try)
    print result
else:
    print "failed to find a valid solution :("
    
