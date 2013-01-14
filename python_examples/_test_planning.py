import trajoptpy as traj
import openravepy as rave
import numpy as np
import json
import atexit
# plan an arm traj
import jds_utils.math_utils as mu
import jds_utils.func_utils as fu
import rave_kinematics as rk
import persistent


def make_request_skeleton(manip_name, n_steps):
    return {
      "basic_info" : {
        "n_steps" : n_steps,
        "manip" : manip_name,
        "start_fixed" : True
      },
      "costs" : [
      {
        "type" : "joint_vel",
        "name" : "jvel",
        "params": {
          "coeffs" : [1]
        }
      },
      {
        "type" : "collision",
#        "name" : "collision0",
        "params" : {
          "coeffs" : [20],
          "dist_pen" : [0.025]
        }
      }
      ],
      "init_info" : {
      }
    }


######## GLOBAL stuff
def initialize():
    if not hasattr(persistent,"env"):
        persistent.env = rave.Environment()
        #persistent.env.Load("robots/pr2-beta-static.zae")
        persistent.env.Load("data/pr2test2.env.xml")
        persistent.env.StopSimulation()
        atexit.register(rave.RaveDestroy)    
        persistent.robot = persistent.env.GetRobots()[0]
    
######## 
    
    
def test_cart_traj():
    robot.SetDOFValues([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.SetTransform(rave.matrixFromPose([1, 0, 0, 0, 2, 0, 0]))    
    
    manipname = "rightarm"
    manip = robot.GetManipulator(manipname)

    n_steps = 5
    prob_desc = make_request_skeleton(manipname, n_steps);
    prob_desc["basic_info"]["start_fixed"] = False
    decimation = 1
    pts = mu.linspace2d([2.5,-.3,1],[2.5,.3,1],n_steps/decimation)
    for (i,pt) in enumerate(pts):
        prob_desc["costs"].append(
        {"type" : "pose",
        "name" : "posexxx",
        "params" : {
            "pos_coeffs" : [1,1,1],
            "rot_coeffs" : [0,0,0],
            "xyz" : pt.tolist(),
            "wxyz" : [.8,.6,0,0],
            "link" : "r_gripper_tool_frame",
            "timestep" : i*decimation            
        }}
        )
    prob_desc["init_info"]["type"] = "stationary"
    
    s = json.dumps(prob_desc)

    traj.SetInteractive(True);
    prob = traj.ConstructProblem(s, env)
    return traj.OptimizeProblem(prob)
        
def test_position_base(pose_targ):
    
    wxyz_targ = pose_targ[:4]
    xyz_targ = pose_targ[4:]
    
    angle_init = np.random.rand() * 2*np.pi
    x_init = xyz_targ[0] - .5*np.cos(angle_init)
    y_init = xyz_targ[1] - .5*np.sin(angle_init)
    dofvals_init = np.r_[np.zeros(7), 0, x_init, y_init, angle_init]
    
    robot.SetDOFValues([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.SetTransform(rave.matrixFromPose([1, 0, 0, 0, -3.4, -1.4, 0.05]))
    manipname = "rightarm+torso_lift_joint+base"
    n_steps = 1
    prob_desc = make_request_skeleton(manipname, n_steps);
    prob_desc["basic_info"]["start_fixed"] = False
    prob_desc["costs"].append(
    {"type" : "pose",
    "name" : "pose",
    "params" : {
        "pos_coeffs" : [10,10,10],
        "rot_coeffs" : [0,0,0],
        "xyz" : xyz_targ,
        "wxyz" : wxyz_targ,
        "link" : "r_gripper_tool_frame",
    }})

    prob_desc["init_info"]["type"] = "given_traj"
    prob_desc["init_info"]["data"] = [dofvals_init.tolist()]
    s = json.dumps(prob_desc)

    traj.SetInteractive(True);
    prob = traj.ConstructProblem(s, env)
    return traj.OptimizeProblem(prob)
    
def test_drive_base(pose_targ):
    
    wxyz_targ = pose_targ[:4]
    xyz_targ = pose_targ[4:]
    

    
    robot.SetDOFValues([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    robot.SetTransform(rave.matrixFromPose([1, 0, 0, 0, -3.4, -1.4, 0.05]))
    
    robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices(), robot.GetLink("torso_lift_link").GetIndex()], 
                        rave.DOFAffine.X + rave.DOFAffine.Y + rave.DOFAffine.RotationAxis, [0,0,1])
    
    angle_init = np.random.rand() * 2*np.pi
    x_init = xyz_targ[0] - .5*np.cos(angle_init)
    y_init = xyz_targ[1] - .5*np.sin(angle_init)
    dofvals_init = np.r_[np.zeros(7), 0, x_init, y_init, angle_init]    
    #start = robot.
    
    start = robot.GetActiveDOFValues();
    end = dofvals_init
    n_steps = 10;
    init_traj = mu.linspace2d(start, end, n_steps)

    
    manipname = "active"
    prob_desc = make_request_skeleton(manipname, n_steps);
    prob_desc["basic_info"]["start_fixed"] = True
    prob_desc["costs"].append(
    {"type" : "pose",
    "name" : "pose",
    "params" : {
        "pos_coeffs" : [10,10,10],
        "rot_coeffs" : [0,0,0],
        "xyz" : xyz_targ,
        "wxyz" : wxyz_targ,
        "link" : "r_gripper_tool_frame",
    }})

    prob_desc["init_info"]["type"] = "given_traj"
    angle = np.linalg.norm(rave.axisAngleFromRotationMatrix(robot.GetTransform()))

    prob_desc["init_info"]["data"] = [row.tolist() for row in init_traj]
    s = json.dumps(prob_desc)

    traj.SetInteractive(True);
    prob = traj.ConstructProblem(s, env)
    return traj.OptimizeProblem(prob)
        

if __name__ == "__main__":
    initialize()
    env = persistent.env
    robot = persistent.robot
    result = test_cart_traj()
    #result = test_drive_base([1, 0, 0, 0, -2.199, -0.138, 0.840675])
    
