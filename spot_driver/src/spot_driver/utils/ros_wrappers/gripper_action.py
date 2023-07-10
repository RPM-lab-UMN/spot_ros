from spot_msgs.msg import GripperAction, GripperResult, GripperFeedback
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import numpy as np
import actionlib
import threading
import rospy
from quaternion import quaternion, as_rotation_matrix

class GripperActionServer:
    def __init__(self, ros_wrapper, action_name, feedback_rate=5):

        self._server = actionlib.SimpleActionServer(
                action_name,
                GripperAction,
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
        raise NotImplementedError("Must be implemented by subclass")

    def _handle_feedback(self):
        while not rospy.is_shutdown() and self._running:
            f = GripperFeedback(self.task_wrapper.feedback)
            self._server.publish_feedback(f)
            rospy.Rate(self.feedback_rate).sleep()

    def _ros_pose_to_mat(self, pose:Pose):
        T = np.eye(4)
        p = pose.position
        T[:3, 3] = np.array([p.x, p.y, p.z])
        o = pose.orientation
        q = quaternion(o.w, o.x, o.y, o.z)
        T[:3, :3] = as_rotation_matrix(q)[:3, :3]
        return T

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



class MultiGraspActionServer(GripperActionServer):
    def __init__(self, ros_wrapper, action_name, feedback_rate=5):
        super().__init__(ros_wrapper, action_name, feedback_rate)

    def handler(self, goal):
        rospy.loginfo(f"gripper_action.py/MultiGraspActionServer/handler: {goal}")
        # TODO return task wrapper task call
        return self.task_wrapper.multigrasp(goal)
