##############################################
# Date: 09/15/2023
# Author: Xun Tu

# This the file for the abstract, generic definition of an action 
# server builder related to SPOT
##############################################

import numpy as np
import actionlib
import rospy
from geometry_msgs.msg import Pose

from quaternion import quaternion, as_rotation_matrix

class ActionServerBuilder:
    def __init__(self, action, result, feedback, feedback_rate, action_name, ros_wrapper):
        self._action = action
        self._result = result
        self._feedback = feedback

        self._server = actionlib.SimpleActionServer(
            action_name,
            action,
            execute_cb=self._handle_action,
            auto_start=False,
            )
        self._server.start()

        self._feedback_thread = None
        self.feedback_rate = feedback_rate
        self._running = False
        self.task_wrapper = ros_wrapper.task_wrapper
        self.ros_wrapper = ros_wrapper

    def handler(self, goal):
        raise NotImplementedError("Must be implemented by a subclass")

    def _handle_feedback(self):
        while not rospy.is_shutdown() and self._running:
            f = self._feedback(self.task_wrapper.feedback)
            self._server.publish_feedback(f)
            rospy.Rate(self.feedback_rate).sleep()

    def _ros_pose_to_mat(self, pose: Pose):
        T = np.eye(4)
        p = pose.position
        T[:3, 3] = np.array([p.x, p.y, p.z])
        o = pose.orientation
        q = quaternion(o.w, o.x, o.y, o.z)
        T[:3, :3] = as_rotation_matrix(q)[:3, :3]
        return T

    def _handle_action(self, goal):
        raise NotImplementedError("Must be implemented by a subclass")