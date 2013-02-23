import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scene_name")
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--save_tmp")
args = parser.parse_args()

import numpy as np, os.path as osp
import cloudprocpy,trajoptpy
cloud_orig = cloudprocpy.readPCDXYZ(osp.join(trajoptpy.bigdata_dir,args.scene_name,"cloud.pcd"))
dof_vals = np.loadtxt(osp.join(trajoptpy.bigdata_dir, args.scene_name, "dof_vals.txt"))
T_w_k = np.loadtxt(osp.join(trajoptpy.bigdata_dir, args.scene_name, "kinect_frame.txt"))

cloud_filt = cloudprocpy.fastBilateralFilter(cloud_orig, 15, .05)
big_mesh = cloudprocpy.meshOFM(cloud_filt, 3, .1)
if args.save_tmp: big_mesh.save("/tmp/big_mesh.ply")
simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, .01)
if args.save_tmp: simple_mesh.save("/tmp/simple_mesh.ply")

import openravepy, trajoptpy
import trajoptpy.make_kinbodies as mk

env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobots()[0]
viewer = trajoptpy.GetViewer(env)
handles = []

#handles.append(env.plot3(cloud_orig.to2dArray().dot(T_w_k.T),3))
#viewer.Idle()

robot.SetDOFValues(dof_vals)


#T_w_k = robot.GetLink("narrow_stereo_gazebo_r_stereo_camera_optical_frame").GetTransform()

#handles.append(env.plot3(cloud_orig.to2dArray().dot(T_w_k.T),3))
#viewer.Idle()



mk.create_trimesh(env, simple_mesh.getCloud().to2dArray().dot(T_w_k.T), np.array(simple_mesh.getFaces()), name="simple_mesh")
viewer.Idle()

convex_meshes = cloudprocpy.convexDecompHACD(simple_mesh)
for (i,mesh) in enumerate(convex_meshes):
    name = "mesh%i"%i
    pts_cam = mesh.getCloud().to2dArray()
    verts = pts_cam.dot(T_w_k.T)[:,:3]
    mk.create_trimesh(env, verts, mesh.getTriangles(), name=name)
    env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetAmbientColor(np.random.rand(3))
    env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetDiffuseColor(np.random.rand(3))
    #handles.append(env.drawtrimesh(mesh.getVertices(), mesh.getTriangles()))
viewer.Idle()


"""

import trajoptpy
import openravepy as rave
import numpy as np
import json
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku
from trajoptpy.check_traj import traj_is_safe


def position_base_request(robot, link_name, xyz_targ, quat_targ):
        
    request = {        
        "basic_info" : {
            "n_steps" : 10,
            "manip" : "active",
            "start_fixed" : True
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
            "type" : "stationary"
        }
    }
    
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
        
    
### Env setup ####
env = rave.Environment()
env.StopSimulation()
env.Load(ENV_FILE)
robot = env.GetRobots()[0]
robot.SetDOFValues([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
robot.SetTransform(rave.matrixFromPose([1, 0, 0, 0, -3.4, -1.4, 0.05]))

robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices(), 
                          robot.GetJoint("torso_lift_joint").GetDOFIndex()], 
                    rave.DOFAffine.X + rave.DOFAffine.Y + rave.DOFAffine.RotationAxis, [0,0,1])
##################
    
success = False
    
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import trajoptpy
import openravepy as rave
import numpy as np
import json
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku
from trajoptpy.check_traj import traj_is_safe

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
    trajoptpy.SetInteractive(args.interactive);
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
    raise Exception("fail")
    
"""