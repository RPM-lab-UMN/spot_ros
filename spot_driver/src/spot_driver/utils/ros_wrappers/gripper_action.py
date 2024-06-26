####################################################
# Author: Xun Tu
# Date: 09/15/2023
#
# This file should contain the ROS-related items for gripper actions
###################################################


from spot_msgs.msg import GripperAction, GripperResult, GripperFeedback

from std_msgs.msg import Header

import rospy
import threading
from .ActionServer import ActionServerBuilder



class GripperActionServer(ActionServerBuilder):
    def __init__(self, ros_wrapper, action_name, feedback_rate=5):
        # ros_wrapper here refers to the "biggest" wrapper of SPOT ROS servers defined in spot_ros
        super().__init__(GripperAction, GripperResult, GripperFeedback, feedback_rate, action_name, ros_wrapper)

    def _handle_action(self, goal):
        rospy.logdebug("Received goal: " + str(goal))

        # Check requirements
        goal = self.ros_wrapper._transform_pose_to_body_frame(goal)
        rospy.logdebug(f"transformed goal to: {goal.pose}")
        
        # Start feedback thread
        self._feedback_thread = threading.Thread(target=self._handle_feedback, args=())
        self._running = True
        self._feedback_thread.start()

        # Run action
        try:
            self.handler(goal)
            self._server.set_succeeded(
                GripperResult(
                    success=True,
                    message=self.task_wrapper.feedback
                )
            )
        except Exception as e:
            self._server.set_aborted(
                GripperResult(
                    success=False,
                    message = self.task_wrapper.feedback + '\n' + str(e)
                )
            )

        # Stop feedback thread
        self._running = False
        self._feedback_thread.join()


class GraspActionServer(GripperActionServer):
    def __init__(self, ros_wrapper, action_name, feedback_rate=5):
        super().__init__(ros_wrapper, action_name, feedback_rate)

    def handler(self, goal):
        pose = self._ros_pose_to_mat(goal.pose)
        return self.task_wrapper.grasp(pose, goal.header.frame_id)


class MoveActionServer(GripperActionServer):
    def __init__(self, ros_wrapper, action_name, feedback_rate=5):
        super().__init__(ros_wrapper, action_name, feedback_rate)

    def handler(self, goal):
        pose = self._ros_pose_to_mat(goal.pose)
        return self.task_wrapper.move_object(pose, goal.header.frame_id)
