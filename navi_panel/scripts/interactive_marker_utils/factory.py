#!/usr/bin/env python3

# Contains the functions necessary for creating a waypoint marker, abstracting the creation of
# the necessary objects and messages into a singular function call

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

import math
import rospy


def create_waypoint_marker(name, reference_frame='odom', position=[1,1,1], orientation=[0,0,0,1]):
    marker = Marker()
    marker.type = CYLINDER

    int_marker_msg = InteractiveMarker()
    int_marker_msg.frame_id = reference_frame
    pos = int_marker_msg.pose.position
    pos.x, pos.y, pos.z = tuple(position)
    ori = int_marker_msg.pose.orientation
    ori.x, ori.y, ori.z = tuple(orientation)
    int_marker_msg.name = name
    int_marker_msg.description = "description"

    marker_control = InteractiveMarkerControl()
    marker_control.interaction_mode = BUTTON
    marker_control.always_visible = True
    marker_control.name = "waypoint-click-button"
    marker_control.markers.append(marker)
    int_marker_msg.controls.append(marker_control)

    return int_marker_msg
