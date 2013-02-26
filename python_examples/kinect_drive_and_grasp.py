import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scene_name")
parser.add_argument("geom_type", choices=["mesh","cd","spheres", "boxes"])
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import numpy as np, os.path as osp
import cloudprocpy,trajoptpy,openravepy
import json
import trajoptpy.math_utils as mu
import trajoptpy.kin_utils as ku
import trajoptpy.make_kinbodies as mk
from trajoptpy.check_traj import traj_is_safe

cloud_orig = cloudprocpy.readPCDXYZ(osp.join(trajoptpy.bigdata_dir,args.scene_name,"cloud.pcd"))
dof_vals = np.loadtxt(osp.join(trajoptpy.bigdata_dir, args.scene_name, "dof_vals.txt"))
T_w_k = np.loadtxt(osp.join(trajoptpy.bigdata_dir, args.scene_name, "kinect_frame.txt"))

def remove_floor(cloud):
    notfloor = get_xyz_world_frame(cloud)[:,2] > .1
    cloud = cloudprocpy.maskFilter(cloud, notfloor, True)
    return cloud

def generate_mesh(cloud):
    cloud = cloudprocpy.fastBilateralFilter(cloud, 15, .05)
    cloud = remove_floor(cloud)
    big_mesh = cloudprocpy.meshOFM(cloud, 3, .1)
    simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, .02)
    return simple_mesh

def get_xyz_world_frame(cloud):
    xyz1 = cloud.to2dArray()
    xyz1[:,3] = 1
    return xyz1.dot(T_w_k.T)[:,:3]

env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
robot = env.GetRobots()[0]
viewer = trajoptpy.GetViewer(env)
handles = []


if 1:
    import IPython
    IPython.lib.inputhook.set_inputhook(viewer.Step)
    qw,qx,qy,qz,px,py,pz = np.loadtxt(osp.join(trajoptpy.bigdata_dir, args.scene_name, "pose_target.txt"))


robot.SetDOFValues(dof_vals)

if args.geom_type == "mesh":
    mesh = generate_mesh(cloud_orig)
    mk.create_trimesh(env, get_xyz_world_frame(mesh.getCloud()), np.array(mesh.getFaces()), name="simple_mesh")
elif args.geom_type == "cd":
    big_mesh = generate_mesh(cloud_orig)
    convex_meshes = cloudprocpy.convexDecompHACD(big_mesh,30)
    for (i,mesh) in enumerate(convex_meshes):
        name = "mesh%i"%i
        verts = get_xyz_world_frame(mesh.getCloud())
        mk.create_trimesh(env, verts, mesh.getTriangles(), name=name)
        env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetAmbientColor(np.random.rand(3))
        env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetDiffuseColor(np.random.rand(3))
elif args.geom_type == "spheres":
    raise Exception("don't use spheres. inifromspheres is too slow")
    cloud_nofloor = remove_floor(cloud_orig)
    cloud_ds = cloudprocpy.downsampleCloud(cloud_nofloor, .04)
    mk.create_spheres(env, get_xyz_world_frame(cloud_ds), .02)
elif args.geom_type == "boxes":
    cloud_nofloor = remove_floor(cloud_orig)
    cloud_ds = cloudprocpy.downsampleCloud(cloud_nofloor, .04)
    mk.create_boxes(env, get_xyz_world_frame(cloud_ds), .02)
    


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
        },
        {
            "type" : "joint_vel",
            "params" : {"coeffs" : [1]}
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
        

robot.SetActiveDOFs(np.r_[robot.GetManipulator("rightarm").GetArmIndices(), 
                          robot.GetJoint("torso_lift_joint").GetDOFIndex()], 
                    openravepy.DOFAffine.X + openravepy.DOFAffine.Y + openravepy.DOFAffine.RotationAxis, [0,0,1])
    
request = position_base_request(robot, "r_gripper_tool_frame", (px,py,pz), (qw,qx,qy,qz))
s = json.dumps(request)
trajoptpy.SetInteractive(args.interactive);
prob = trajoptpy.ConstructProblem(s, env)
result = trajoptpy.OptimizeProblem(prob)
if check_result(result, robot):
    print "success!"
else:
    print "fail :("
