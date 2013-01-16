#!/usr/bin/env python

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest("interactive_markers")
import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

import geometry_msgs.msg as gm

from random import random
from math import sin

#menu_handler = MenuHandler()
#br = None
#counter = 0
name2marker = {}

_server = None
def getServer():
    global _server
    if _server is None:
        _server = InteractiveMarkerServer("basic_controls")        
    return _server

def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame" )
    counter += 1

def processFeedback( feedback ):
    server = getServer()
    
    name2marker[feedback.marker_name].process(feedback)    
    
    ##s = "Feedback from marker '" + feedback.marker_name
    ##s += "' / control '" + feedback.control_name + "'"

    ##mp = ""
    ##if feedback.mouse_point_valid:
        ##mp = " at " + str(feedback.mouse_point.x)
        ##mp += ", " + str(feedback.mouse_point.y)
        ##mp += ", " + str(feedback.mouse_point.z)
        ##mp += " in frame " + feedback.header.frame_id

    ##if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        ##rospy.loginfo( s + ": button click" + mp + "." )
    ##elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        ##rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    ##elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        ##rospy.loginfo( s + ": pose changed")
### TODO
###          << "\nposition = "
###          << feedback.pose.position.x
###          << ", " << feedback.pose.position.y
###          << ", " << feedback.pose.position.z
###          << "\norientation = "
###          << feedback.pose.orientation.w
###          << ", " << feedback.pose.orientation.x
###          << ", " << feedback.pose.orientation.y
###          << ", " << feedback.pose.orientation.z
###          << "\nframe: " << feedback.header.frame_id
###          << " time: " << feedback.header.stamp.sec << "sec, "
###          << feedback.header.stamp.nsec << " nsec" )
    ##elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        ##rospy.loginfo( s + ": mouse down" + mp + "." )
    ##elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        ##rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def alignMarker( feedback ):
    pose = feedback.pose

    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5

    rospy.loginfo( feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                                                                     str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

def rand( min_, max_ ):
    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation
class MarkerControl(object):
    def process(self, feedback):
        abstract


class SixDOFControl(MarkerControl):
    def __init__(self, fixed, xyz_init, xyzw_init):
        self.xyz = xyz_init
        self.xyzw = xyzw_init
        

        server = getServer()
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/base_link"
        int_marker.pose.position = gm.Point(*xyz_init)
        int_marker.pose.orientation = gm.Quaternion(*xyzw_init)
        int_marker.scale = 1
    
        int_marker.name = "simple_6dof"
        int_marker.description = "Simple 6-DOF Control"
    
        # insert a box
        makeBoxControl(int_marker)
    
        if fixed:
            int_marker.name += "_fixed"
            int_marker.description += "\n(fixed orientation)"
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)
    
        server.insert(int_marker, processFeedback)
        server.applyChanges()
        name2marker[int_marker.name] = self
        
    def process(self, feedback):
        pos = feedback.pose.position
        self.xyz = [pos.x, pos.y, pos.z]
        quat = feedback.pose.orientation
        self.xyzw = [quat.x, quat.y, quat.z, quat.w]
        
def makeRandomDofMarker():
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1

    int_marker.name = "6dof_random_axes"
    int_marker.description = "6-DOF\n(Arbitrary Axes)"

    makeBoxControl(int_marker)

    control = InteractiveMarkerControl()

    for i in range(3):
        control.orientation.w = rand(-1,1)
        control.orientation.x = rand(-1,1)
        control.orientation.y = rand(-1,1)
        control.orientation.z = rand(-1,1)
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(copy.deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    server.applyChanges()
def makeViewFacingMarker():
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1

    int_marker.name = "view_facing"
    int_marker.description = "View Facing 6-DOF"

    # make a control that rotates around the view axis
    control = InteractiveMarkerControl()
    control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation.w = 1
    control.name = "rotate"
    int_marker.controls.append(control)

    # create a box in the center which should not be view facing,
    # but move in the camera plane.
    control = InteractiveMarkerControl()
    control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.independent_marker_orientation = True
    control.name = "move"
    control.markers.append( makeBox(int_marker) )
    control.always_visible = True
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

def makeQuadrocopterMarker():
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1

    int_marker.name = "quadrocopter"
    int_marker.description = "Quadrocopter"

    makeBoxControl(int_marker)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    int_marker.controls.append(copy.deepcopy(control))
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

def makeChessPieceMarker():
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1

    int_marker.name = "chess_piece"
    int_marker.description = "Chess Piece\n(2D Move + Alignment)"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    int_marker.controls.append(copy.deepcopy(control))

    # make a box which also moves in the plane
    control.markers.append( makeBox(int_marker) )
    control.always_visible = True
    int_marker.controls.append(control)

    # we want to use our special callback function
    server.insert(int_marker, processFeedback)

    # set different callback for POSE_UPDATE feedback
    server.setCallback(int_marker.name, alignMarker, InteractiveMarkerFeedback.POSE_UPDATE )

def makePanTiltMarker():
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1

    int_marker.name = "pan_tilt"
    int_marker.description = "Pan / Tilt"

    makeBoxControl(int_marker)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    control.orientation_mode = InteractiveMarkerControl.INHERIT
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)

def makeMenuMarker():
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1

    int_marker.name = "context_menu"
    int_marker.description = "Context Menu\n(Right Click)"

    # make one control using default visuals
    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description="Options"
    control.name = "menu_only_control"
    int_marker.controls.append(copy.deepcopy(control))

    # make one control showing a box
    marker = makeBox( int_marker )
    control.markers.append( marker )
    control.always_visible = True
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )

def makeMovingMarker():
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/moving_frame"
    int_marker.pose.position.y = -3.0 * marker_pos
    marker_pos += 1
    int_marker.scale = 1

    int_marker.name = "moving"
    int_marker.description = "Marker Attached to a\nMoving Frame"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(copy.deepcopy(control))

    control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    control.always_visible = True
    control.markers.append( makeBox(int_marker) )
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)


def oldmain():
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()
    
    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert( "First Entry", callback=processFeedback )
    menu_handler.insert( "Second Entry", callback=processFeedback )
    sub_menu_handle = menu_handler.insert( "Submenu" )
    menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )

    make6DofMarker( False )
    make6DofMarker( True )
    makeRandomDofMarker( )
    makeViewFacingMarker( )
    makeQuadrocopterMarker( )
    makeChessPieceMarker( )
    makePanTiltMarker( )
    makeMenuMarker( )
    makeMovingMarker( )

    server.applyChanges()

    rospy.spin()

