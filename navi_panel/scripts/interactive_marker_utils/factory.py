#!/usr/bin/env python3

# Contains the functions necessary for creating a waypoint marker, abstracting the creation of
# the necessary objects and messages into a singular function call

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

import math
import rospy

UNSELECTED_R = 0.45
UNSELECTED_G = 0.06
UNSELECTED_B = 0.67
SELECTED_R = 0.05
SELECTED_G = 0.45
SELECTED_B = 0.98


def create_waypoint_marker(wp_id, reference_frame='odom', position=[1,1,1], orientation=[0,0,0,1]):
    """
    Creates an interactive, clickable marker for use in Rviz for representing graph navigation
    waypoints for Hachi.

    Args:
        wp_id: The string of the waypoint ID. Used as both the marker name and description text.
        reference_frame: The reference frame within which the marker is located
        position: The initial position of the marker
        orientation: The initial orientation of the marker
    """

    # Builds the marker in top-down order. The interactive marker message is the highest-level object,
    # and is what ultimately is sent to the marker server for creation. It contains controls, which
    # in-turn contain marker objects, which are the actual 3D shapes that you see in Rviz.

    int_marker_msg = InteractiveMarker()
    int_marker_msg.header.frame_id = reference_frame
    pos = int_marker_msg.pose.position
    pos.x, pos.y, pos.z = tuple(position)
    ori = int_marker_msg.pose.orientation
    ori.x, ori.y, ori.z, ori.w = tuple(orientation)
    int_marker_msg.name = "marker:" + wp_id
    int_marker_msg.description = wp_id

    marker_control = InteractiveMarkerControl()
    marker_control.interaction_mode = InteractiveMarkerControl.MENU
    marker_control.always_visible = True
    marker_control.name = "waypoint-click-menu"
    int_marker_msg.controls.append(marker_control)

    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.color.r = UNSELECTED_R
    marker.color.g = UNSELECTED_G
    marker.color.b = UNSELECTED_B
    marker.color.a = 1.0
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.6
    marker_control.markers.append(marker)

    return int_marker_msg
