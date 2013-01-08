import trajoptpy
import openravepy as rave
import numpy as np
import json
import atexit # avoid segfault  at exit
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku

def position_base_request(robot, link_name, xyz_targ, quat_targ):
        
    request = {
        "basic_info" : {
            "n_steps" : 1,
            "manip" : "rightarm+torso_lift_joint+base",
            "start_fixed" : False
        },
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


    angle_init = np.random.rand() * 2*np.pi
    x_init = xyz_targ[0] - .5*np.cos(angle_init)
    y_init = xyz_targ[1] - .5*np.sin(angle_init)
    lower,upper = robot.GetActiveDOFLimits()
    lower = np.clip(lower, -np.pi, np.pi)
    upper = np.clip(upper, -np.pi, np.pi)
    rands = np.random.rand(len(lower))
    joint_vals = lower*rands + upper*(1-rands)
    dofvals_init = np.r_[.3*joint_vals[:-3], x_init, y_init, angle_init]

    request["init_info"]["type"] = "given_traj"
    request["init_info"]["data"] = [dofvals_init.tolist()]

    
    return request


def main():
        
    ### Parameters ###
    ENV_FILE = "data/pr2test1.env.xml"
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
    robot.SetDOFValues([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.SetTransform(rave.matrixFromPose([1, 0, 0, 0, -3.4, -1.4, 0.05]))
    robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices(), 
                              robot.GetJoint("torso_lift_joint").GetDOFIndex()], 
                        rave.DOFAffine.X + rave.DOFAffine.Y + rave.DOFAffine.RotationAxis, [0,0,1])
    
    ##################
    
    for i_try in xrange(100):
        request = position_base_request(robot, LINK_NAME, XYZ_TARGET, QUAT_TARGET)
        s = json.dumps(request)
        print "REQUEST:",s
        trajoptpy.SetInteractive(INTERACTIVE);
        prob = trajoptpy.ConstructProblem(s, env)
        result = trajoptpy.OptimizeProblem(prob)
        if result.GetConstraints()[0][1] < 1e-3:
            break
    print "succeeded on try %i"%(i_try)
    print result
if __name__ == "__main__":
    main()