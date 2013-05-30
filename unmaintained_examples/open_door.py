import trajoptpy
import openravepy as rave
import numpy as np
import json
from trajoptpy import convex_soup
import atexit

def pose_traj_request(robot, hmats):
    n_steps = len(hmats)
        
    request = {
        "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "leftarm+rightarm+torso_lift_joint+base",
            "start_fixed" : False
        },
        "costs" : [
        {
            "type" : "joint_vel",            
            "params": {"coeffs" : np.r_[np.ones(15), 10*np.ones(3)].tolist()}
        },            
        {
            "type" : "collision",
            "params" : {"coeffs" : [.1],"dist_pen" : [0.01]}
        }
        ],
        "constraints" : [
        ],
        "init_info" : {
            "type" : "stationary"
        }
    }
    
    poses = rave.poseFromMatrices(hmats)
    xyzs = poses[:,4:7]
    quats = poses[:,0:4]
    
    for i in xrange(1, n_steps-1):
        print "waypoint xyz", xyzs[i]
        waypoint_cnt = {
            "type" : "pose",
            "name" : "waypoint_pose",
            "params" : {
                "xyz" : list(xyzs[i]),
                "wxyz" : list(quats[i]),
                "link" : "l_gripper_tool_frame",
                "pos_coeffs" : [1,1,1],
                "rot_coeffs" : [1,1,1],
                "timestep" : i
            }}
        request["constraints"].append(waypoint_cnt)    

    return request    


env = rave.RaveGetEnvironment(1)
if env is None:
    env = rave.Environment()
    env.StopSimulation()
    env.Load('../data/pr2-door.env.xml')
    atexit.register(rave.RaveDestroy)

robot = env.GetRobot("pr2")
robot.SetDOFValues(np.zeros(robot.GetDOF()))
robot_transform = np.eye(4)
robot_transform[0,3] = -1.2
robot.SetTransform(robot_transform)
rgj = robot.GetJoint("l_gripper_l_finger_joint")
robot.SetDOFValues([.2], [rgj.GetDOFIndex()])
tlj = robot.GetJoint("torso_lift_joint")
robot.SetDOFValues([tlj.GetLimits()[-1]], [tlj.GetDOFIndex()])

def draw_ax(T, size, env, handles):
    print T
    p0 = T[:3,3]
    xax, yax, zax = T[:3,:3].T*size
    width = size/10.
    handles.append(env.drawarrow(p0, p0+xax, width, [1,0,0]))
    handles.append(env.drawarrow(p0, p0+yax, width, [0,1,0]))
    handles.append(env.drawarrow(p0, p0+zax, width, [0,0,1]))

import IPython
viewer = trajoptpy.GetViewer(env)
IPython.lib.inputhook.set_inputhook(viewer.Step)

plot_handles = []

door = env.GetKinBody("door")
handle = door.GetLink("handle")

draw_ax(handle.GetGlobalMassFrame(), .1, env, plot_handles)
viewer.Idle()

door.SetDOFValues([np.pi/3], [1])

angles = np.linspace(0, -np.pi*.4, 10)
T_world_handle = np.eye(4); 
T_world_handle[:3,3] += [0.06, -.39120+.02, .952-.04]; 
T_world_handle[:3,:3] = rave.rotationMatrixFromAxisAngle([1,0,0], np.pi/3)
T_world_hinge = np.eye(4); 
T_world_hinge[:3,3] += door.GetJoint("doorhinge").GetAnchor()
T_hinge_handle = np.linalg.inv(T_world_hinge).dot(T_world_handle)

hmats = [T_world_hinge.dot(rave.matrixFromAxisAngle([0,0,1],a)).dot(T_hinge_handle) for a in angles]

request = pose_traj_request(robot, hmats)
s = json.dumps(request)
print "REQUEST:",s
trajoptpy.SetInteractive(True);
prob = trajoptpy.ConstructProblem(s, env)
result = trajoptpy.OptimizeProblem(prob)
traj = result.GetTraj()

traj_samples = range(10)
handles = []
for step in traj_samples:
    robot.SetActiveDOFValues(traj[step])
    door.SetDOFValues([-angles[step]], [0])
    robot_plot = viewer.PlotKinBody(robot)
    handles.append(robot_plot)
    #robot_plot.SetTransparency(.35)
    door_plot = viewer.PlotKinBody(door)
    handles.append(door_plot)
    door_plot.SetTransparency(.5)
robot.SetActiveDOFValues(traj[0])    
door.SetDOFValues([0],[0])