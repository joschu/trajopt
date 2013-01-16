import cloudprocpy
import trajoptpy
import openravepy as rave
import numpy as np
import json
from trajoptpy import convex_soup
import atexit
import brett2.ros_utils as ru
from brett2.PR2 import PR2
import basic_controls
import rospy

if rospy.get_name() == "/unnamed": rospy.init_node('cloud_drive_reach_live')



def drive_to_reach_request(robot, link_name, xyz_targ, quat_targ):
        
    request = {
        "basic_info" : {
            "n_steps" : 10,
            "manip" : "rightarm+base+r_gripper_l_finger_joint",
            "start_fixed" : True
        },
        "costs" : [
        {
            "type" : "joint_vel",
            "params": {"coeffs" : [1]}
        },            
        {
            "type" : "collision",
            "params" : {"coeffs" : [10],"dist_pen" : [0.01]}
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

pr2 = PR2.create()
env = pr2.env
robot = pr2.robot
    

print "waiting for point cloud on /drop/points"
import sensor_msgs.msg as sm
pc = rospy.wait_for_message("/drop/points", sm.PointCloud2)
print "ok"
pc_tf = ru.transformPointCloud2(pc, pr2.tf_listener, "base_footprint", pc.header.frame_id)
    

xyz, bgr = ru.pc2xyzrgb(pc_tf)
xyz = xyz.reshape(-1,3).astype('float32')
cloud = cloudprocpy.PointCloudXYZ()
cloud.from2dArray(xyz)
cloud = cloudprocpy.boxFilter(cloud, -1,5,-3,3,.1,2)
#aabb = robot.GetLink("base_link").ComputeAABB()
#(xmin,ymin,zmin) = aabb.pos() - aabb.extents()
#(xmax,ymax,zmax) = aabb.pos() + aabb.extents()
#cloud = cloudprocpy.boxFilterNegative(cloud, xmin,xmax,ymin,ymax,zmin,zmax)
cloud = cloudprocpy.downsampleCloud(cloud, .015)
convex_soup.create_convex_soup(cloud, env)




marker = basic_controls.SixDOFControl( False , [1,0,1], [0,0,0,1])
raw_input("now choose target pose with interactive markers. press enter when done")

xyz = marker.xyz
wxyz = np.r_[marker.xyzw[3], marker.xyzw[:3]].tolist()


##################

request = drive_to_reach_request(robot, "r_gripper_tool_frame", xyz, wxyz)
s = json.dumps(request)
print "REQUEST:",s
trajoptpy.SetInteractive(True);
prob = trajoptpy.ConstructProblem(s, env)
result = trajoptpy.OptimizeProblem(prob)