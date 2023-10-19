from spot_msgs.msg import ObstacleMoveAction, ObstacleMoveResult, ObstacleMoveFeedback
from spot_msgs.msg import FindGraspPointAction, FindGraspPointGoal, FindGraspPointFeedback, FindGraspPointResult
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
import actionlib
from cv_bridge import CvBridge, CvBridgeError

class ObstacleMoveActionServer(ActionServerBuilder):
    def __init__(self, ros_wrapper, action_name, feedback_rate=5):
        # ros_wrapper here refers to the "biggest" wrapper of SPOT ROS servers defined in spot_ros
        super().__init__(ObstacleMoveAction, ObstacleMoveResult, ObstacleMoveFeedback,
                          feedback_rate, action_name, ros_wrapper)
        self.bridge = CvBridge()
        self._find_grasp_point_client = actionlib.SimpleActionClient("find_grasp_point", FindGraspPointAction)

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
        
        handle_res = self.handler(req)
        if (handle_res == "SUCCEED"):
            self._server.set_succeeded(
                ObstacleMoveResult(
                    success= True,
                    message="SUCCEED"
                )
            )
        elif ( handle_res == "NO_GRASP"):
            self._server.set_succeeded(
                ObstacleMoveResult(
                    success= True,
                    message="NO_GRASP"
                )
            )
        else:
            self._server.set_aborted(
                ObstacleMoveResult(
                    success= False,
                    message = "ERROR"
                )
            )
        
        

        # Stop feedback thread
        self._running = False
        self._feedback_thread.join()

    def handler(self, req):
        # Part I: command the gripper to grasp a place on the chair

        #grasp_res = self.task_wrapper.take_image_grasp()
        image, depth_image = self.task_wrapper.take_image_for_grasp()

        # Convert the image into cv2 type
        image_cv2 = self.task_wrapper._convert_to_cv2(image)
        
        
        image_ros = self.bridge.cv2_to_imgmsg(image_cv2, "rgb8")
        self.ros_wrapper.logger.info("Create image message!")
        
        request = FindGraspPointGoal(image_ros, "DINO")
        self._find_grasp_point_client.wait_for_server()

        self.ros_wrapper.logger.info("The server is found!")
        self._find_grasp_point_client.send_goal(request)
        self._find_grasp_point_client.wait_for_result()
        res = self._find_grasp_point_client.get_result()
        pick_x = res.pick_x
        pick_y = res.pick_y

        if(res.success == False):
            # If an Error occurred
            return "ERROR"
        elif (pick_x == 0.0 and pick_y == 0.0):
            # If everything goes fine, yet the system fails to find a good grasp point
            # Do nothing and return directly
            self.ros_wrapper.logger.info("No good grasp point Found...")
            return "NO_GRASP"
        # elif (depth_image[int(pick_x), int(pick_y)] > 1500 or depth_image[int(pick_x), int(pick_y)] == 0):
        #     # If everything goes fine, but the depth constraint is violated
        #     # Do nothing and return directly
        #     self.ros_wrapper.logger.info("The grasp point is too far or too close ... " 
        #                                  + str(depth_image[int(pick_x), int(pick_y)]))
        #     return "SUCCEED"

        self.ros_wrapper.logger.info("Grasp point Found!")
        
        self.ros_wrapper.logger.info("( + " + str(pick_x) + " , " + str(pick_y) + ")")
        grasp_res = self.task_wrapper.take_pick_grasp(pick_x, pick_y, image)
        #grasp_res = self.task_wrapper.take_pick_grasp_manual(pick_x, pick_y, image)
        if(grasp_res == False): 
            # The grasp attempt failed!
            self.task_wrapper._end_grasp()
            return "NO_GRASP"
        
        # Part II: move the robot to the destination
        # Potential bug in the provided codes:
        # Assuming the z-coordinate for the obstalce is 0...
        #
        #
        # Also, so far, just command the robot to move 1.2m rightward...
        # spot_target_pose is in body frame
        # because I want to keep the relative orientation between the gripper
        # and the body. The relative orientation should not change as much as 
        # possible, unless the chair is heavy and the robot has to adapt to the weight

        # If the odom pose of spot is used, when converting it back to body pose,
        # the error is large
        self.ros_wrapper.logger.info("Grasp Finished! Calculating...")
        spot_curr_location = req.spot_location
        spot_target_location = req.spot_destination
        time.sleep(2) # Wait until the grasp is stable     
        spot_curr_pose = self.task_wrapper._ros_pose_to_bd_se3(spot_curr_location.pose)
        spot_target_pose = self.task_wrapper._ros_pose_to_bd_se3(spot_target_location.pose)

        if spot_target_pose.position.x != 0:
            self.ros_wrapper.logger.info("Hmmm... the robot is going back a little bit...")
            spot_go_back_pose = bdSE3Pose(spot_target_pose.position.x, 0, 0, bdQuat())
            # If we need the robot to go back for some steps
            gripper_target_pose = self.task_wrapper._predict_gripper_pose(spot_go_back_pose, spot_target_location.header.frame_id)
            self.ros_wrapper.logger.info("Send arm impedance & trajectory command")
            self.task_wrapper._drag_arm_impedance(gripper_target_pose, spot_go_back_pose, spot_target_location.header.frame_id)

            self.ros_wrapper.logger.info("Now the robot is going rightward/leftward!")
            spot_move_pose = bdSE3Pose(0, spot_target_pose.position.y, 0, bdQuat())
            # Ready to go rightward/leftward
            gripper_target_pose = self.task_wrapper._predict_gripper_pose(spot_move_pose, spot_target_location.header.frame_id)
            self.ros_wrapper.logger.info("Send arm impedance & trajectory command")
            self.task_wrapper._drag_arm_impedance(gripper_target_pose, spot_move_pose, spot_target_location.header.frame_id)

        else:
            gripper_target_pose = self.task_wrapper._predict_gripper_pose(spot_target_pose, spot_target_location.header.frame_id)
            self.ros_wrapper.logger.info("Send arm impedance & trajectory command")

            # The conversion of the pose from odom frame back to body frame may have a large error
            # Currently spot_target_pose is specified in body frame
            self.task_wrapper._drag_arm_impedance(gripper_target_pose, spot_target_pose, spot_target_location.header.frame_id)
        time.sleep(2)
        self.ros_wrapper.logger.info("Obstacle Moved! Stow the arm ...")
        self.task_wrapper._end_grasp()
        # Part III: move back to the original place

        self.ros_wrapper.logger.info("Arm stowed! Going back to original place ...")
        if spot_target_pose.position.x != 0:
            self.ros_wrapper.logger.info("Two stages to go back!")
            spot_first_pose = bdSE3Pose(0, -spot_target_pose.position.y, 0, bdQuat())
            self.task_wrapper._go_along_trajectory(spot_first_pose, 4, BODY_FRAME_NAME)
            spot_second_pose = bdSE3Pose(-spot_target_pose.position.x, 0, 0, bdQuat())
            self.task_wrapper._go_along_trajectory(spot_second_pose, 4, BODY_FRAME_NAME)
        else:
            self.task_wrapper._go_along_trajectory(spot_curr_pose, 4, spot_curr_location.header.frame_id)
        self.ros_wrapper.logger.info("Back to original place!")

        return "SUCCEED"
