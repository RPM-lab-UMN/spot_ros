#!/usr/bin/env python3

# Contains the functions necessary for creating a waypoint marker, abstracting the creation of
# the necessary objects and messages into a singular function call

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs import *

import math
import rospy

UNSELECTED_R = 0.45
UNSELECTED_G = 0.06
UNSELECTED_B = 0.67
SELECTED_R = 0.05
SELECTED_G = 0.45
SELECTED_B = 0.98
ARROW_R = 0.2
ARROW_G = 0.8
ARROW_B = 0.2


def create_waypoint_marker(wp_id, pose, reference_frame='odom'):
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
    int_marker_msg.pose = pose
    int_marker_msg.name = wp_id
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


def create_edge_marker(start, end, reference_frame="odom"):
    """
    Creates an arrow marker pointing from the marker at :start: to the marker at :end:

    Args:
        start: the interactive marker the arrow starts at
        end: the interactive marker the arrow ends at
    """

    int_marker_msg = InteractiveMarker()
    int_marker_msg.header.frame_id = reference_frame
    int_marker_msg.name = "edge:" + start.waypoint_id + "->" + end.waypoint_id

    marker_control = InteractiveMarkerControl()
    marker_control.interaction_mode = InteractiveMarkerControl.NONE
    marker_control.always_visible = True
    marker_control.name = "edge-control"
    int_marker_msg.controls.append(marker_control)

    marker = Marker()
    marker.type = Marker.ARROW
    marker.points = [start.pose.position, end.pose.position]
    marker.color.r = ARROW_R
    marker.color.g = ARROW_G
    marker.color.b = ARROW_B
    marker.color.a = 1.0
    marker.scale.x = 0.1    # shaft diameter
    marker.scale.y = 0.15    # head diameter
    marker.scale.z = 0.0    # head length (when nonzero)
    marker_control.markers.append(marker)

    return int_marker_msg
