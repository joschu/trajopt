import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku

env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
env.Load("../data/table.xml")

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting

robot = env.GetRobots()[0]
joint_start = [-1.832, -0.332, -1.011, -1.437, -1.1  , -2.106,  3.074]
robot.SetDOFValues(joint_start, robot.GetManipulator('rightarm').GetArmIndices())

quat_target = [0.98555024,  0.12101977,  0.10129305, -0.06151951] # wxyz
xyz_target = [0.65540048, -0.34836676,  0.44726639]
hmat_target = openravepy.matrixFromPose( np.r_[quat_target, xyz_target] )

# BEGIN ik
manip = robot.GetManipulator("rightarm")
init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_tool_frame",
    filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)
# END ik


request = {
  "basic_info" : {
    "n_steps" : 10,
    "manip" : "rightarm", # see below for valid values
    "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
  },
  "costs" : [
  {
    "type" : "joint_vel", # joint-space velocity cost
    "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
    # Also valid: "coeffs" : [7,6,5,4,3,2,1]
  },
  {
    "type" : "continuous_collision",
    "name" :"cont_coll", # shorten name so printed table will be prettier
    "params" : {
      "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
      "dist_pen" : [0.025] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
    }
  }
  ],
  "constraints" : [
  # BEGIN pose_constraint
  {
    "type" : "pose", 
    "params" : {"xyz" : xyz_target, 
                "wxyz" : quat_target, 
                "link": "r_gripper_tool_frame",
                # "timestep" : 9
                # omitted because timestep = n_steps-1 is default
                # "pos_coeffs" : [1,1,1], # omitted because that's default
                "rot_coeffs" : ([0,0,0] if args.position_only else [1,1,1])
                }
                 
  }
  # END pose_constraint
  ],
  # BEGIN init
  "init_info" : {
      "type" : "straight_line", # straight line in joint space.
      "endpoint" : init_joint_target.tolist() # need to convert numpy array to list
  }
  # END init
}
s = json.dumps(request) # convert dictionary into json-formatted string
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
result = trajoptpy.OptimizeProblem(prob) # do optimization
print result

from trajoptpy.check_traj import traj_is_safe
prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
assert traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free

# Now we'll check to see that the final constraint was satisfied
robot.SetActiveDOFValues(result.GetTraj()[-1])
posevec = openravepy.poseFromMatrix(robot.GetLink("r_gripper_tool_frame").GetTransform())
quat, xyz = posevec[0:4], posevec[4:7]

quat *= np.sign(quat.dot(quat_target))
if args.position_only:
    assert (quat - quat_target).max() > 1e-3
else:
    assert (quat - quat_target).max() < 1e-3

