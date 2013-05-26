import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import openravepy
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku, trajoptpy.math_utils as mu
import os.path as osp

env = openravepy.Environment()
env.StopSimulation()
env.Load(osp.join(trajoptpy.data_dir, "carbot.xml"))

trajoptpy.SetInteractive(args.interactive) # pause every iteration, until you press 'p'. Press escape to disable further plotting

robot = env.GetRobots()[0]
robot.SetActiveDOFs([], 11)

start = [0,0,0]
goal = [0, 5.,np.pi]
# midpt = [1.,.5,0]
# midpt = (np.array(start) + np.array(goal))/2

n_steps = 19
# init_traj = np.r_[mu.linspace2d(start, midpt, (n_steps+1)/2), mu.linspace2d(midpt, goal, (n_steps+1)/2)[1:]]
init_traj = mu.linspace2d(start, goal, n_steps)



request = {
  "basic_info" : {
    "n_steps" : n_steps,
    "manip" : "active", 
    "start_fixed" : True 
  },
  "costs" : [
  {
    "type" : "joint_vel",
    "params": {"coeffs" : [1,1,1]} 
  },
  # {
  #   "type" : "collision",
  #   "params" : {
  #     "coeffs" : [20],
  #     "dist_pen" : [0.025] 
  #   }
  # }
  ],
  "constraints" : [
  # BEGIN pose_target
  {
    "type" : "joint", 
    "params" : {"vals" : goal}                 
  },
  {
      "type" :"joint_velocity",
      "params" : {"vals" : [np.abs(np.array(start) - np.array(goal))*2].tolist()}
  }
  
  ],
  # "init_info" : {
  #     "type" : "straight_line",
  #     "endpoint" : goal
  # }
  "init_info" : {
      "type" : "given_traj",
      "data" : [row.tolist() for row in init_traj]
  }
  
}
s = json.dumps(request) 
prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem

def f(a):
    x0,y0,t0,x1,y1,t1 = a
    return 100*((t0+t1)/2 - np.arctan2(y1-y0, x1-x0))

for t1 in xrange(1,n_steps):    
    prob.AddConstraint(f, [(t,j) for t in (t1-1, t1) for j in xrange(3)], "EQ", "car%i"%t)

result = trajoptpy.OptimizeProblem(prob) # do optimization
print result
