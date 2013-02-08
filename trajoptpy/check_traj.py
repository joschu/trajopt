import trajoptpy.math_utils as mu
import openravepy as rave
import numpy as np
from time import time
def check_traj(traj, manip, n=100):
    traj_up = mu.interp2d(np.linspace(0,1,n), np.linspace(0,1,len(traj)), traj)
    robot = manip.GetRobot()
    ss = rave.RobotStateSaver(robot)
    arm_inds = manip.GetArmIndices()
    env = robot.GetEnv()
    collision = False
    col_times = []
    for (i,row) in enumerate(traj_up):
        robot.SetDOFValues(row, arm_inds)
        col_now = env.CheckCollision(robot)
        if col_now: 
            collision = True
            col_times.append(i)
    if col_times: print "collision at timesteps", col_times      
    else: print "no collisions"
    return collision
