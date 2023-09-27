from spot_msgs.msg import ObstacleMoveAction, ObstacleMoveResult, ObstacleMoveFeedback

from std_msgs.msg import Header
from geometry_msgs.msg import TwistWithCovarianceStamped, Twist, Pose, PoseStamped, Point
from geometry_msgs.msg import Quaternion as QuatMessage
import rospy
import threading
from bosdyn.client.frame_helpers import (
    ODOM_FRAME_NAME,
    BODY_FRAME_NAME,
    HAND_FRAME_NAME
)
from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose
from bosdyn.client.math_helpers import Quat as bdQuat
from .ActionServer import ActionServerBuilder
import time
import numpy as np

class ObstacleMoveActionServer(ActionServerBuilder):
    def __init__(self, ros_wrapper, action_name, feedback_rate=5):
        # ros_wrapper here refers to the "biggest" wrapper of SPOT ROS servers defined in spot_ros
        super().__init__(ObstacleMoveAction, ObstacleMoveResult, ObstacleMoveFeedback,
                          feedback_rate, action_name, ros_wrapper)

    def _handle_action(self, req):
        # goal is in the type of ObstacleMoveAction
        rospy.logdebug("Received ObstacleMove request at: " + str(req.spot_location))

        # Check requirements
        spot_destination = self.ros_wrapper._transform_pose_to_body_frame(req.spot_destination)
        rospy.logdebug(f"Move the SPOT to: {spot_destination}")
        
        # Start feedback thread
        self._feedback_thread = threading.Thread(target=self._handle_feedback, args=())
        self._running = True
        self._feedback_thread.start()

        # Run action
        
        self.handler(req)
        self._server.set_succeeded(
            ObstacleMoveResult(
                success=True,
                message="Succeeded in Removing the obstacle/Need to remove obstacle!"
            )
        )
        '''
        except Exception as e:
            self._server.set_aborted(
                ObstacleMoveResult(
                    success=False,
                    message = "Exception Occurred as: " + '\n' + str(e)
                )
            )
        '''
        

        # Stop feedback thread
        self._running = False
        self._feedback_thread.join()

    def handler(self, req):
        # Part I: command the gripper to grasp a place on the chair
        grasp_res = self.task_wrapper.take_image_grasp()

        if(grasp_res == False): #user doesn't apply a grasp
            return
        
        # Part II: move the robot to the destination
        # Potential bug in the provided codes:
        # Assuming the z-coordinate for the obstalce is 0...
        #
        # Currently, I am thinking about changing the style into
        # observing the spot position destination
        #
        # Also, so far, just command the robot to move 1m leftward...
        # First attempt: read the current gripper pose, and move the gripper horizontally
        # TODO: replace it with the desired obstacle target location
        self.ros_wrapper.logger.info("Grasp Finished! Calculating...")
        spot_curr_location = req.spot_location
        spot_target_location = req.spot_destination
        time.sleep(2) # Wait until the grasp is stable     
        spot_curr_pose = self.task_wrapper._ros_pose_to_bd_se3(spot_curr_location.pose)
        spot_target_pose = self.task_wrapper._ros_pose_to_bd_se3(spot_target_location.pose)


        gripper_target_pose = self.task_wrapper._predict_gripper_pose(spot_target_pose, spot_target_location.header.frame_id)
        self.ros_wrapper.logger.info("Send arm impedance & trajectory command")
        self.task_wrapper._drag_arm_impedance(gripper_target_pose, spot_target_pose, spot_target_location.header.frame_id)
        time.sleep(2)
        self.ros_wrapper.logger.info("Obstacle Moved! Stow the arm ...")
        self.task_wrapper._end_grasp()
        # Part III: move back to the original place

        self.ros_wrapper.logger.info("Arm stowed! Going back to original place ...")
        self.task_wrapper._go_along_trajectory(spot_curr_pose, 10, spot_curr_location.header.frame_id)
        self.ros_wrapper.logger.info("Back to original place!")
