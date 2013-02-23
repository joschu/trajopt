import argparse
parser = argparse.ArgumentParser()
parser.add_argument("scene_name")
parser.add_argument("--dry_run", action="store_true")
args = parser.parse_args()

"""
On robot:
roslaunch openni_launch kinect_frames.launch
"""


import os, os.path as osp
import openravepy, cloudprocpy, trajoptpy
import numpy as np
#import bottleneck


scene_dir = osp.join(trajoptpy.bigdata_dir, args.scene_name)
if not args.dry_run: os.mkdir(scene_dir)

import rospy
rospy.init_node("acquire_cloud_data",disable_signals=True)

from brett2.PR2 import PR2
pr2 = PR2()
robot = pr2.robot;

pr2.tf_listener.waitForTransform("/base_footprint","/camera_rgb_optical_frame", rospy.Time(0),rospy.Duration(1))
(x,y,z), (qx,qy,qz,qw) = pr2.tf_listener.lookupTransform("/base_footprint","/camera_rgb_optical_frame", rospy.Time(0))
T_w_k = openravepy.matrixFromPose([qw,qx,qy,qz,x,y,z])
if not args.dry_run: np.savetxt(osp.join(scene_dir, "kinect_frame.txt"), T_w_k)

dof_vals = robot.GetDOFValues()
if not args.dry_run: np.savetxt(osp.join(scene_dir,"dof_vals.txt"), dof_vals)


grabber=cloudprocpy.CloudGrabber()


xyzrgb = grabber.getXYZRGB()
if not args.dry_run: xyzrgb.save(osp.join(scene_dir,"cloud.pcd"))

#clouds = []
#grabber.startXYZ()
#for i in xrange(10):
    #xyzrgb = grabber.getXYZ()
    #clouds.append(xyzrgb.to3dArray())
#grabber.stop()
#avg_cloud = cloudprocpy.CloudXYZ()
#avg_cloud.from3dArray(bottleneck.nanmean(np.array(clouds), axis=0))
#if not args.dry_run: avg_cloud.save(osp.join(scene_dir,"cloud.pcd"))
