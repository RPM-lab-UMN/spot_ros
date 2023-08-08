from geometry_msgs.msg import PoseStamped, Pose

import numpy as np
import rospy

from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose
from bosdyn.client.math_helpers import Quat as bdQuat


class WaypointPoseSubscriber(object):
    """
    Subscribes to the pose topic and updates the appropriate markers' positions
    according to the messages on that topic.

    !!! ASSUMES THAT THE MESSAGES ARE IN A SPECIFIC FORMAT CONTAINING THE MARKER NAME !!!
    """
    def __init__(self, topic, marker_server):
        """
        Args:
            topic: the name of the topic to subscribe to
            marker_server: the marker server to send updates to
        """

        self._sub = rospy.Subscriber(topic, Pose, self)
        self._server = marker_server

        def __call__()
