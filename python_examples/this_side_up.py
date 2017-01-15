import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--diffmethod", choices=["numerical", "analytic"], default="analytic")
parser.add_argument("--use_cost", action="store_true")
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku

env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting

robot = env.GetRobots()[0]
manip = robot.GetManipulator("rightarm")
robot.SetActiveManipulator(manip)
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
    robot, iktype=openravepy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()

xyz_target = [.6,-.6,1]


n_steps = 15
hmat_target = openravepy.matrixFromPose( np.r_[(0,1,0,0), xyz_target] )

init_joint_target = ku.ik_for_link(hmat_target, manip, "r_gripper_tool_frame",
    filter_options = openravepy.IkFilterOptions.CheckEnvCollisions)


request = {
  "basic_info" : {
    "n_steps" : n_steps,
    "manip" : "rightarm", 
    "start_fixed" : True 
  },
  "costs" : [
  {
    "type" : "joint_vel",
    "params": {"coeffs" : [1]} 
  },
  {
    "type" : "collision",
    "params" : {
      "coeffs" : [20],
      "dist_pen" : [0.025] 
    }
  }
  ],
  "constraints" : [
  # BEGIN pose_target
  {
    "type" : "pose", 
    "params" : {"xyz" : xyz_target, 
                "wxyz" : [1,0,0,0],  # unused
                "link": "r_gripper_tool_frame",
                "rot_coeffs" : [0,0,0],
                "pos_coeffs" : [10,10,10]
                }
                 
  },
  #END pose_target
  #BEGIN vel
  {
    "type" : "cart_vel",
    "name" : "cart_vel",
    "params" : {
        "max_displacement" : .05,
        "first_step" : 0,
        "last_step" : n_steps-1, #inclusive
        "link" : "r_gripper_tool_frame"
    },
  }
  #END vel  
  ],
  "init_info" : {
      "type" : "straight_line",
      "endpoint" : init_joint_target.tolist()
  }
}
s = json.dumps(request) 
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem

tool_link = robot.GetLink("r_gripper_tool_frame")
local_dir = np.array([0.,0.,1.])

arm_inds = manip.GetArmIndices()
arm_joints = [robot.GetJointFromDOFIndex(ind) for ind in arm_inds]

# BEGIN python_funcs
def f(x):
    robot.SetDOFValues(x, arm_inds, False)
    return tool_link.GetTransform()[:2,:3].dot(local_dir)
def dfdx(x):
    robot.SetDOFValues(x, arm_inds, False)
    world_dir = tool_link.GetTransform()[:3,:3].dot(local_dir)
    return np.array([np.cross(joint.GetAxis(), world_dir)[:2] for joint in arm_joints]).T.copy()
# END python_funcs

if args.use_cost:
# BEGIN add_costs
    for t in xrange(1,n_steps):    
        if args.diffmethod == "numerical":
            prob.AddErrorCost(f, [(t,j) for j in xrange(7)], "ABS", "up%i"%t)
        elif args.diffmethod == "analytic":
            prob.AddErrorCost(f, dfdx, [(t,j) for j in xrange(7)], "ABS", "up%i"%t)
# END add_costs
else: #use constraint
# BEGIN add_constraints
    for t in xrange(1,n_steps):    
        if args.diffmethod == "numerical":
            prob.AddConstraint(f, [(t,j) for j in xrange(7)], "EQ", "up%i"%t)
        elif args.diffmethod == "analytic":
            prob.AddConstraint(f, dfdx, [(t,j) for j in xrange(7)], "EQ", "up%i"%t)
# END add_constraints

result = trajoptpy.OptimizeProblem(prob) # do optimization
assert [viol <= 1e-4 for (_name,viol) in result.GetConstraints()]
