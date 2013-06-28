import argparse
parser = argparse.ArgumentParser(description="e.g. python kinect_drive_and_reach.py green_table cd --interactive")
parser.add_argument("scene_name")
parser.add_argument("geom_type", choices=["mesh","cd","spheres", "boxes"])
parser.add_argument("--interactive", action="store_true")
args = parser.parse_args()

import numpy as np, os.path as osp
import cloudprocpy,trajoptpy,openravepy
import json
import trajoptpy.make_kinbodies as mk
from trajoptpy.check_traj import traj_is_safe

assert openravepy.__version__ >= "0.9"
    

def remove_floor(cloud):
    notfloor = get_xyz_world_frame(cloud)[:,2] > .1
    cloud = cloudprocpy.maskFilter(cloud, notfloor, True)
    return cloud

# BEGIN generate_mesh
def generate_mesh(cloud):
    cloud = cloudprocpy.fastBilateralFilter(cloud, 15, .05) # smooth out the depth image
    cloud = remove_floor(cloud) # remove points with low height (in world frame)
    big_mesh = cloudprocpy.meshOFM(cloud, 3, .1) # use pcl OrganizedFastMesh to make mesh
    simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, .02) # decimate mesh with VTK function
    return simple_mesh
# END generate_mesh

def get_xyz_world_frame(cloud):
    xyz1 = cloud.to2dArray()
    xyz1[:,3] = 1
    return xyz1.dot(T_w_k.T)[:,:3]

cloud_orig = cloudprocpy.readPCDXYZ(osp.join(trajoptpy.bigdata_dir,args.scene_name,"cloud.pcd"))
dof_vals = np.loadtxt(osp.join(trajoptpy.bigdata_dir, args.scene_name, "dof_vals.txt"))
T_w_k = np.loadtxt(osp.join(trajoptpy.bigdata_dir, args.scene_name, "kinect_frame.txt"))
qw,qx,qy,qz,px,py,pz = np.loadtxt(osp.join(trajoptpy.bigdata_dir, args.scene_name, "pose_target.txt"))

env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")

robot = env.GetRobots()[0]
robot.SetDOFValues(dof_vals)

viewer = trajoptpy.GetViewer(env)

try:
    import IPython
    IPython.lib.inputhook.set_inputhook(viewer.Step)
except Exception: 
    pass


# Plotting original point cloud
cloud_orig_colored = cloudprocpy.readPCDXYZRGB(osp.join(trajoptpy.bigdata_dir,args.scene_name,"cloud.pcd"))
rgbfloats = cloud_orig_colored.to2dArray()[:,4]
rgb0 = np.ndarray(buffer=rgbfloats.copy(),shape=(480*640,4),dtype='uint8')
xyz=get_xyz_world_frame(cloud_orig)
goodinds = np.isfinite(xyz[:,0])
cloud_handle = env.plot3(xyz[goodinds,:], 2,(rgb0[goodinds,:3][:,::-1])/255. )
viewer.Idle()
del cloud_handle
######


handles = []

# BEGIN addtoenv
if args.geom_type == "mesh":
    mesh = generate_mesh(cloud_orig)
    mesh_body = mk.create_trimesh(env, get_xyz_world_frame(mesh.getCloud()), np.array(mesh.getFaces()), name="simple_mesh")
    mesh_body.SetUserData("bt_use_trimesh", True) # Tell collision checker to use the trimesh rather than the convex hull of it
elif args.geom_type == "cd":
    big_mesh = generate_mesh(cloud_orig)
    convex_meshes = cloudprocpy.convexDecompHACD(big_mesh,30)
    for (i,mesh) in enumerate(convex_meshes):
        name = "mesh%i"%i
        verts = get_xyz_world_frame(mesh.getCloud())
        mk.create_trimesh(env, verts, mesh.getTriangles(), name=name)
        randcolor = np.random.rand(3)
        env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetAmbientColor(randcolor)
        env.GetKinBody(name).GetLinks()[0].GetGeometries()[0].SetDiffuseColor(randcolor)
# END  addtoenv        
elif args.geom_type == "spheres":
    raise Exception("don't use spheres--there's a performance issue")
    cloud_nofloor = remove_floor(cloud_orig)
    cloud_ds = cloudprocpy.downsampleCloud(cloud_nofloor, .04)
    mk.create_spheres(env, get_xyz_world_frame(cloud_ds), .02)
elif args.geom_type == "boxes":
    cloud_nofloor = remove_floor(cloud_orig)
    cloud_ds = cloudprocpy.downsampleCloud(cloud_nofloor, .04)
    mk.create_boxes(env, get_xyz_world_frame(cloud_ds), .02)
    

def full_body_drive_and_reach(link_name, xyz_targ, quat_targ):
        
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
    
request = full_body_drive_and_reach("r_gripper_tool_frame", (px,py,pz), (qw,qx,qy,qz))
s = json.dumps(request)
trajoptpy.SetInteractive(args.interactive)
prob = trajoptpy.ConstructProblem(s, env)
result = trajoptpy.OptimizeProblem(prob)
if check_result(result, robot):
    print "success!"
else:
    print "fail :("
