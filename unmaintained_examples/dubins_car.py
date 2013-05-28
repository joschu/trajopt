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
aff = openravepy.DOFAffine
robot.SetActiveDOFs([], aff.X | aff.Y | aff.RotationAxis)

start = [0,0,0]
goal = [2,1,0]
# midpt = [.1,2.5,np.pi/2]
# midpt = (np.array(start) + np.array(goal))/2

n_steps = 19
# init_traj = np.r_[mu.linspace2d(start, midpt, (n_steps+1)/2), mu.linspace2d(midpt, goal, (n_steps+1)/2)[1:]]
init_traj = mu.linspace2d(start, goal, n_steps)

max_variations = (np.abs(np.array(goal) - np.array(start)))*2
max_variations = np.clip(max_variations, np.pi, 100.);
max_variations /= n_steps-1

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
      "type" :"joint_vel_limits",
      "params" : {"vals" : max_variations.tolist(), "first_step":0, "last_step": n_steps-1}
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


import math
def g(a):
    x0,y0,t0,x1,y1,t1 = a
    dx = x1-x0
    dy = y1-y0
    dl = math.hypot(dx, dy)
    ang = (t0+t1)/2
    err = np.array([math.cos(ang) - dx/dl, math.sin(ang) - dy/dl])
    return err;
def f(a):
    return 1e3**(g(a)**2).sum()

for t1 in xrange(1,n_steps):    
    prob.AddConstraint(g, [(t,j) for t in (t1-1, t1) for j in xrange(3)], "EQ", "carcnt%i"%t)
    prob.AddCost(f, [(t,j) for t in (t1-1, t1) for j in xrange(3)], "carcost%i"%t)

result = trajoptpy.OptimizeProblem(prob) # do optimization
print result
