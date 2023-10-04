#################################################################################
# Date: 09/15/2023
# Author of this Header: Xun Tu
#
# Recommendations on writing programs in this file:
# 
# This is the wrapper file to for general-purpose long, complicated tasks for SPOT.
# Typically, a lot of extra logic other than sending requests and reading 
# responses is required. Ideally, you would like to figure out the complex
# parts in this file, and call the functions in spot_wrapper or set up clients
# directly to do the final direct communications with the robot. 
# 
# should NOT call ROS servers deployed on SPOT

# Example: if you want to move the robot to a place after you study
# the point cloud, it would be a good choice to define the labor-intensive parts, 
# such as processing the point cloud data into readable formats, filter out the outliers, or
# figuring out the mathematical steps to find the target place, within this file

# Note: for the more complicated or more specific tasks, such as use services
# provided by graphNav to do navigation, you could also create individual files
###############################################################################

# Spot wrapper
from ..spot_wrapper import SpotWrapper
from ..ros_helpers import (get_numpy_data_type,
                           expand_data_by_rle_count,
                           unpack_grid,
                           local_grid_value)

# Messages
from bosdyn.api import (robot_command_pb2,
                        geometry_pb2,
                        image_pb2,
                        mobility_command_pb2, 
                        basic_command_pb2,
                        manipulation_api_pb2,
                        arm_command_pb2)

from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.api.arm_command_pb2 import (ArmCommand, ArmCartesianCommand)
from bosdyn.api.basic_command_pb2 import (ArmDragCommand)
from bosdyn.api.robot_command_pb2 import (RobotCommand )
from bosdyn.api.synchronized_command_pb2 import (SynchronizedCommand)

from bosdyn.client.math_helpers import SE2Pose as bdSE2Pose
from bosdyn.client.math_helpers import SE3Pose as bdSE3Pose
from bosdyn.client.math_helpers import Quat    as bdQuat



# Clients
from bosdyn.client.manipulation_api_client import ManipulationApiClient

from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, 
                                         ODOM_FRAME_NAME, 
                                         HAND_FRAME_NAME,
                                         VISION_FRAME_NAME,
                                         get_se2_a_tform_b,
                                         get_vision_tform_body,
                                         get_a_tform_b)

from bosdyn.client.robot_command import (block_until_arm_arrives, block_for_trajectory_cmd)
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_state import RobotStateClient

from bosdyn.client.robot_command import RobotCommandBuilder as CmdBuilder

# Others
from bosdyn.util import seconds_to_duration

import numpy as np
import logging
import time
import cv2

# Temp
from bosdyn.client import math_helpers

g_image_click = None
g_image_display = None
def cv_mouse_callback(event, x, y, flags, param):
    global g_image_click, g_image_display
    clone = g_image_display.copy()
    if event == cv2.EVENT_LBUTTONUP:
        g_image_click = (x, y)
    else:
        # Draw some lines on the image.
        #print('mouse', x, y)
        color = (30, 30, 30)
        thickness = 2
        image_title = 'Click to grasp'
        height = clone.shape[0]
        width = clone.shape[1]
        cv2.line(clone, (0, y), (width, y), color, thickness)
        cv2.line(clone, (x, 0), (x, height), color, thickness)
        cv2.imshow(image_title, clone)

class SpotTaskWrapper:

    def __init__(self, spot: SpotWrapper, logger:logging.Logger=None):
        self.spot = spot
        self._robot = spot._robot
        self._init_logger(logger)
        assert self._robot.has_arm(), "Robot requires an arm to run this example."
        self._manip_cli = self._robot.ensure_client(ManipulationApiClient.default_service_name)
        self.default_ref_frame = ODOM_FRAME_NAME  # VISION_FRAME_NAME, ODOM_FRAME_NAME
        self._log.info('Task Wrapper Initialized.')

    def _init_logger(self, logger:logging.Logger=None):
        class LogHandler(logging.Handler):
            def __init__(self, ref): 
                super().__init__()
                self._ref = ref
            def emit(self, record): 
                self._ref._string_feedback = str(record)
        self._log_handler = LogHandler(self)
        if logger is None: logger = logging.getLogger(__name__)
        logger.addHandler(self._log_handler)
        self._log = logger

    @property
    def feedback(self): return self._string_feedback


    '''
    Helper functions
    '''
    def _pose_np_to_bd(self, pose:np.array, se3=False):
        '''Change input np array into bd pose'''
        if pose.shape == (3,3):
            pose = bdSE2Pose.from_matrix(pose)
            if se3: pose = pose.get_closest_se3_transform()
        else: 
            pose = bdSE3Pose.from_matrix(pose)
        return pose    
     
    def _pose_bd_to_vectors(self, pose:bdSE3Pose):
        '''Change into bd pose into vectors (pos, rot)'''
        pos = [pose.position.x, pose.position.y, pose.position.z]
        rot = [pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z]
        return pos, rot

    def _offset_pose(self, pose:bdSE3Pose, distance, axis):
        '''Calculate the offset along the direction defined by axis'''
        all_axis = {'x':0, 'y':1, 'z':2}
        dir = np.eye(3)[all_axis[axis]] 
        offset = np.eye(4)
        offset[0:3, 3] = -dir * distance
        offset = bdSE3Pose.from_matrix(offset)
        return pose * offset
    
    def _to_bd_se3(self, pose, ref, se3=True):
        '''Changes input pose into a bd se3 pose.'''
        if isinstance(pose, np.ndarray):
            pose = self._pose_np_to_bd(pose, se3=se3)
        pose = self.spot._transform_bd_pose(pose, ref, self.default_ref_frame)
        return pose
    def _ros_pose_to_bd_se3(self, pose):
        '''Changes input ros pose to bdSE3 pose'''
        identity = np.eye(4)
        identity[0, 3] = pose.position.x
        identity[1, 3] = pose.position.y
        identity[2, 3] = pose.position.z
        bd = bdSE3Pose.from_matrix(identity)

        bd.rotation.x = pose.orientation.x
        bd.rotation.y = pose.orientation.y
        bd.rotation.z = pose.orientation.z
        bd.rotation.w = pose.orientation.w

        return bd
    def _get_gripper_initial_pose(self, y_offset, frame_name = "odom"):
        return self.spot._transform_bd_pose(bdSE3Pose(0, y_offset, 0, bdQuat()), HAND_FRAME_NAME, frame_name)
        
    '''
    Move the robot to a a desired pose
    '''
    def go_to(self, pose, relative_frame:str, 
              distance:float=0.0, dir_axis:str='x', 
              up_axis:str='z', blocking=True):
        '''Moves the robot to a desired pose.'''
        if isinstance(pose, np.ndarray):
            pose = self._pose_np_to_bd(pose, se3=True)
        pose = self._offset_pose(pose, distance, dir_axis)
        pos, rot = pose.position, pose.rotation

        heading = rot.to_roll() if up_axis == 'x'\
             else rot.to_pitch() if up_axis == 'y'\
             else rot.to_yaw()        
    
        # TODO: handle this response (ask Bahaa)
        # Notice that we only consider the (x,y) coordinates for the body of the robot
        self.spot.trajectory_cmd(
            goal_x=pos.x, goal_y=pos.y, 
            goal_heading=heading,
            cmd_duration=10, 
            reference_frame=relative_frame,
            blocking=blocking,
        )

    '''
    Release the grasp and stow the arm
    '''
    def _end_grasp(self):
        self.spot.arm_stow()
        self.spot.gripper_close()

    '''
    Grasp the object defined by "pose", the target pose of the gripper
    '''
    def grasp(self, pose, reference_frame:str, **kwargs):
        if not self.spot.arm_stow()[0]:
            raise Exception('Failed to stow arm.')

        # The target pose is the gripper pose
        self._log.debug(f'Grasping pose {pose} in frame {reference_frame}')
        pose = self._to_bd_se3(pose, reference_frame)
        
        # Command the robot to go to a place close to the gripper pose
        self._log.info('Approaching desired robot pose.')
        self.go_to(pose, self.default_ref_frame, distance=1.0, **kwargs)


        def _to(p, d):
            '''
            Build up the formal command sent to 
            the robot to do the grasping
            p: target pose
            d: duration
            '''
            pos, rot = self._pose_bd_to_vectors(p)
            status, msg = self.spot.hand_pose(pos, rot, 
                                reference_frame=self.default_ref_frame, 
                                duration=d)
            self._log.info(f'status: {msg}')
            if status is False: 
                self._end_grasp()
                raise(Exception('Failed...'))
            
        # Move the gripper to the pre-grasp pose
        pre_grasp = self._offset_pose(pose, 0.25, 'x')
        self._log.info(f'Approaching Pre-grasp...')
        _to(pre_grasp, 1.5)

        # Do the final grasp
        self._log.info('Approaching object...')
        self.spot.gripper_open()
        _to(pose, 1.0)
        self.spot.gripper_close()
        self._log.info('Succeeded')
        return True
    
    '''
    Move the object to the target pose
    '''
    def move_object(self, pose, reference_frame:str, **kwargs):
        '''Commands the robot to move an object to a desired pose.'''
        self._log.debug(f'Moving object to pose {pose} in frame {reference_frame}')
        return self._move_heavy_object(pose, reference_frame)

    '''
    Move the arm to the target pose in the reference frame
    '''
    def _follow_arm_to(self, pose, reference_frame):

        pose = self._to_bd_se3(pose, reference_frame)

        # Create the arm & body command.
        arm_command = CmdBuilder.arm_pose_command(
            pose.x, pose.y, pose.z, 
            pose.rot.w, pose.rot.x, pose.rot.y, pose.rot.z, 
            self.default_ref_frame, 
            2.0)
        follow_arm_command = CmdBuilder.follow_arm_command()
        command = CmdBuilder.build_synchro_command(follow_arm_command, 
                                                   arm_command)

        # Send the request
        cmd_client = self.spot._robot_command_client
        move_command_id = cmd_client.robot_command(command)
        self._log.info('Moving arm to position.')

        block_until_arm_arrives(cmd_client, move_command_id, 6.0)
        self._log.info('Succeeded')
        return True
    

    '''
    Command the robot to go along a trajectory in reference frame
    '''
    def _go_along_trajectory(self, pose, duration_sec = 15.0, reference_frame = "odom"):
        """
        Send a trajectory command to the robot

        Args:
            pose: PoseStamped the robot should go to. Must be in the body frame
            duration: After this duration, the command will time out and the robot will stop

        Returns: (bool, str) tuple indicating whether the command was successfully sent, and a message

        """
        pose = pose.get_closest_se2_transform()
        self._log.info("Building trajectory Command")
        succeeded, _, id = self.spot.trajectory_cmd(
            goal_x=pose.x, goal_y=pose.y,
            goal_heading=pose.angle,
            cmd_duration=duration_sec, 
            reference_frame=reference_frame,
            blocking=False
        )
        self._log.info("Going along the trajectory...")
        if succeeded:
            block_for_trajectory_cmd(self.spot._robot_command_client,
                                     cmd_id=id, 
                                     feedback_interval_secs=0.5, 
                                     timeout_sec=duration_sec,
                                     logger=self._log)
        else: 
            self._log.info('Failed to send trajectory command.')
            return False
        return True


    '''
    Order the robot to take an image and grasp one point on the image
    '''
    def _get_pick_vec(self, image):
        # Clarify the image data type
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
        else:
            dtype = np.uint8
        img = np.fromstring(image.shot.image.data, dtype=dtype)

        # Convert the image into standard raw typeS
        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            img = img.reshape(image.shot.image.rows, image.shot.image.cols)
        else:
            img = cv2.imdecode(img, -1)

        # Show the image to the user and wait for them to click on a pixel
        image_title = 'Click to grasp'
        cv2.namedWindow(image_title)
        cv2.setMouseCallback(image_title, cv_mouse_callback)

        global g_image_click, g_image_display
        g_image_display = img
        cv2.imshow(image_title, g_image_display)
        while g_image_click is None:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == ord('Q'):
                # Quit
                return None

        pick_vec = geometry_pb2.Vec2(x=g_image_click[0], y=g_image_click[1])
        return pick_vec
    

    '''
    Add the grasp constraint for the robot
    '''
    def add_grasp_constraint(self, config, grasp, robot_state_client):
        # There are 3 types of constraints:
        #   1. Vector alignment
        #   2. Full rotation
        #   3. Squeeze grasp
        #
        # You can specify more than one if you want and they will be OR'ed together.

        # For these options, we'll use a vector alignment constraint.
        use_vector_constraint = (config == "top_down") | (config == "horizontal") | (config == "horizontal_side")
        # Specify the frame we're using.
        grasp.grasp_params.grasp_params_frame_name = VISION_FRAME_NAME

        if use_vector_constraint:
            if config == "top_down":
                # Add a constraint that requests that the x-axis of the gripper is pointing in the
                # negative-z direction in the vision frame.

                # The axis on the gripper is the x-axis.
                axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

                # The axis in the vision frame is the negative z-axis
                axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=-1)

            if config == "horizontal_side":
                # Add a constraint that requests that the y-axis of the gripper is pointing in the
                # positive-z direction in the vision frame.  That means that the gripper is constrained to be rolled 90 degrees and pointed at the horizon.

                # The axis on the gripper is the y-axis.            print("Test grasp")

                axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)

                # The axis in the vision frame is the positive z-axis
                axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=1)
            if config == "horizontal":
                self._log.info("Horizontal Grasp!!")
                # Another way to grasp the object
                axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=0, z=1)

                axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=1)

            
            # Add the vector constraint to our proto.
            constraint = grasp.grasp_params.allowable_orientation.add()
            constraint.vector_alignment_with_tolerance.axis_on_gripper_ewrt_gripper.CopyFrom(
                axis_on_gripper_ewrt_gripper)
            constraint.vector_alignment_with_tolerance.axis_to_align_with_ewrt_frame.CopyFrom(
                axis_to_align_with_ewrt_vo)

            # We'll take anything within about 10 degrees for top-down or horizontal grasps.
            constraint.vector_alignment_with_tolerance.threshold_radians = 0.17

        elif config == "45_angle_grasp":
            # Demonstration of a RotationWithTolerance constraint.  This constraint allows you to
            # specify a full orientation you want the hand to be in, along with a threshold.
            #
            # You might want this feature when grasping an object with known geometry and you want to
            # make sure you grasp a specific part of it.
            #
            # Here, since we don't have anything in particular we want to grasp,  we'll specify an
            # orientation that will have the hand aligned with robot and rotated down 45 degrees as an
            # example.

            # First, get the robot's position in the world.
            robot_state = robot_state_client.get_robot_state()
            vision_T_body = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)

            # Rotation from the body to our desired grasp.
            body_Q_grasp = math_helpers.Quat.from_pitch(0.785398)  # 45 degrees
            vision_Q_grasp = vision_T_body.rotation * body_Q_grasp

            # Turn into a proto
            constraint = grasp.grasp_params.allowable_orientation.add()
            constraint.rotation_with_tolerance.rotation_ewrt_frame.CopyFrom(vision_Q_grasp.to_proto())

            # We'll accept anything within +/- 10 degrees
            constraint.rotation_with_tolerance.threshold_radians = 0.17

        elif config == "squeeze":
            # Tell the robot to just squeeze on the ground at the given point.
            constraint = grasp.grasp_params.allowable_orientation.add()
            constraint.squeeze_grasp.SetInParent()
    
    
    '''
    Take an image and try to grasp something based on the image
    '''
    def take_image_grasp(self):
        
        
        # Set up the necessary clients
        robot = self._robot

        assert robot.has_arm(), "Robot requires an arm to run this example."

        # TODO: Command the robot to stand up at first if it is not standing

        # Or you could grab it from spot_wrapper
        image_client = self.spot._image_client

        manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)


        # TODO: acquire from all the image sources, and choose the best one
        image_sources = [   "hand_color_image",
                            "frontleft_fisheye_image",
                            "frontright_fisheye_image"
                        ]


        pick_vec = None
        for image_source in image_sources:
            # Take a picture with a camera
            self._log.info('Getting an image from: ' + image_source)
            image_responses = image_client.get_image_from_sources([image_source])

            if len(image_responses) != 1:
                self._log.info('Got invalid number of images: ' + str(len(image_responses)))
                self._log.info(image_responses)
                assert False

            image = image_responses[0]
            pick_vec = self._get_pick_vec(image) #Use the use defined grasp position
            if pick_vec != None:
                break
        
        if (pick_vec == None):
            return False #If the user doesn't find a good grasp position, skip it
        # Build the proto
        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=pick_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole)
        

        grasp_config = "horizontal"
        # Optionally add a grasp constraint.  This lets you tell the robot you only want top-down grasps or side-on grasps.
        self.add_grasp_constraint(grasp_config, grasp, self.spot._robot_state_client)
        # Ask the robot to pick up the object
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)

        # Send the request
        cmd_response = manipulation_api_client.manipulation_api_command(
            manipulation_api_request=grasp_request)

        # Get feedback from the robot
        while True:
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request)

            self._log.info('Current state: '+ manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))


            if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                break

            time.sleep(0.25)

        
        self._log.info('Finished grasp.')
        cv2.destroyAllWindows()
        global g_image_display, g_image_click
        g_image_click = None
        g_image_display = None
        return True
    
    '''
    The following functions are just the split of take_image_grasp
    '''
    def take_image_for_grasp(self):
        image_client = self.spot._image_client
        image_source = "hand_color_image"
        # Take a picture with a camera
        self._log.info('Getting an image from: ' + image_source)
        image_responses = image_client.get_image_from_sources([image_source])

        if len(image_responses) != 1:
            self._log.info('Got invalid number of images: ' + str(len(image_responses)))
            self._log.info(image_responses)
            assert False

        image = image_responses[0]
        return image
    '''
    Convert the image into cv2 format
    '''
    def _convert_to_cv2(self, image):
        # Clarify the image data type
        if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_DEPTH_U16:
            dtype = np.uint16
        else:
            dtype = np.uint8
        img = np.fromstring(image.shot.image.data, dtype=dtype)

        # Convert the image into standard raw typeS
        if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
            img = img.reshape(image.shot.image.rows, image.shot.image.cols)
        else:
            img = cv2.imdecode(img, -1)

        return img
    '''
    Assume we already have the selected point, try to grasp the point
    Version 1: depend on BD's method in grasping
    '''
    def take_pick_grasp(self, pick_x, pick_y, image):
        
        
        # Set up the necessary clients
        robot = self._robot

        assert robot.has_arm(), "Robot requires an arm to run this example."


        manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)


        pick_vec = geometry_pb2.Vec2(x=pick_x, y=pick_y)
       
        
        # Build the proto
        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=pick_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole)
        

        grasp_config = "none"
        # Optionally add a grasp constraint.  This lets you tell the robot you only want top-down grasps or side-on grasps.
        self.add_grasp_constraint(grasp_config, grasp, self.spot._robot_state_client)
        # Ask the robot to pick up the object
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)

        # Send the request
        cmd_response = manipulation_api_client.manipulation_api_command(
            manipulation_api_request=grasp_request)

        # Get feedback from the robot
        attempts_count = 0
        while True:
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request)

            self._log.info('Current state: '+ manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))


            if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                break
            
            if(attempts_count >= 40):
                self._log.info("Awful! Too many grasp attempts!!")
                return False
            
            attempts_count = attempts_count + 1
            time.sleep(0.25)

        
        self._log.info('Finished grasp.')
        return True
    
    '''
    Assume we already have the selected point, try to grasp the point
    Version 2: approximate the coordinate of the selected pixel point in 3D space,
    move the gripper there, and close the gripper!
    '''
    def take_pick_grasp_manual(self, pick_x, pick_y, image):
        
        
        # Set up the necessary clients
        robot = self._robot

        assert robot.has_arm(), "Robot requires an arm to run this example."

        # Calculate the 3D coordinate of the picked pixel

        # Obtain the assoicated depth image
        image_client = self.spot._image_client
        image_source = "hand_depth_in_hand_color_frame"
        # Capture and save images to disk
        image_responses = image_client.get_image_from_sources(image_source)
        depth_image = np.frombuffer(image_responses[0].shot.image.data, dtype=np.uint16)
        depth_image = depth_image.reshape(image_responses[0].shot.image.rows,
                                image_responses[0].shot.image.cols)
        self._log.info("Debugging Message for depth: ")
        self._log.info("The selected pixel is: " + str(pick_x) + ", " + str(pick_y))
        self._log.info(depth_image[pick_x, pick_y])
        grasp_pose_sensor = bdSE3Pose(pick_x, pick_y, depth_image[pick_x, pick_y], bdQuat())

        vision_tform_grasp = get_a_tform_b(
                        image.shot.transforms_snapshot,
                        BODY_FRAME_NAME,
                        image.shot.frame_name_image_sensor)
        
        gripper_target_pose = vision_tform_grasp * grasp_pose_sensor

        # Move the gripper to the place
        # (Cited from grasp() in this wrapper, but with simplification)
        pos, rot = self._pose_bd_to_vectors(gripper_target_pose)
        self.spot.gripper_open()
        status, msg = self.spot.hand_pose(
                    pos, rot, 
                    reference_frame=BODY_FRAME_NAME,
                    duration = 2
        )
        self._log.info(f'status: {msg}')
        if status is False: 
            self._end_grasp()
            self._log.info("Failed to move the gripper to the pose!")
            return False
        self.spot.gripper_close()
        self._log.info("Gripper Closed!")
        return True



       
        
       

    '''
    Move the robot to the desired pose, while gripper attached to the object
    No force within the arm joints
    '''
    def _drag_arm_to(self, pose:bdSE3Pose, reference_frame, duration_sec=15.0):
        '''Commands the robot to position the robot at the commanded
        pose while leaving the arm commpliant thus allowing it to drag
        objects.
        Args:
            pose: The desired pose of the robot.
            reference_frame: The frame in which the pose is defined.
        returns:
                '''
        self._log.info(f'Building drag command.')
        robot_cmd = RobotCommand(
            synchronized_command = SynchronizedCommand.Request(
                arm_command = ArmCommand.Request(
                    arm_drag_command = ArmDragCommand.Request()
                )
            )
        )
        # Set the claw to apply force        
        robot_cmd = CmdBuilder.claw_gripper_close_command(robot_cmd) 

        pose = pose.get_closest_se2_transform()
        self._log.info(f'Sending Robot Command.')
        succeeded, _, id = self.spot.trajectory_cmd(
            goal_x=pose.x, goal_y=pose.y,
            goal_heading=pose.angle,
            cmd_duration=duration_sec, 
            reference_frame=reference_frame,
            blocking=False,
            build_on_command=robot_cmd
        )
        if succeeded:
            block_for_trajectory_cmd(self.spot._robot_command_client,
                                     cmd_id=id, 
                                     feedback_interval_secs=0.5, 
                                     timeout_sec=duration_sec,
                                     logger=self._log)
        else: 
            self._log.error('Failed to send trajectory command.')
            return False
        return True
    '''
    Predict the target gripper pose given that it's orientation w.r.t 
    the body of the robot is unchanged
    '''
    def _predict_gripper_pose(self, spot_target_pose, target_ref_frame_name = BODY_FRAME_NAME):
        # spot_target_pose: the target pose for the spot to move to
        # target_ref_frame_name: the reference frame where the target pose is specified
        Tf_tree = self.spot._robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
        g = get_a_tform_b(Tf_tree, BODY_FRAME_NAME, HAND_FRAME_NAME)
        self._log.info("Gripper current pose (body frame): ")
        self._log.info(g)


        if (target_ref_frame_name != BODY_FRAME_NAME):
            T = get_a_tform_b(Tf_tree, BODY_FRAME_NAME, target_ref_frame_name)
            # In body frame, how the spot changes its orientation
            spot_target_pose_body = T * spot_target_pose
            self._log.info("Spot target pose (body frame)")
            self._log.info(spot_target_pose_body)

            # The gripper tareget pose in body frame
            gripper_target_pose = g * spot_target_pose_body

            self._log.info("Gripper target pose (body frame): ")
            self._log.info(gripper_target_pose)

            # Convert the gripper target pose back in the reference frame of spot_target_pose
            gripper_target_pose = T.inverse() * gripper_target_pose
        else:
            gripper_target_pose = g

        
        
            

        return gripper_target_pose
    
    
    '''
    Move the obstacle to the pose, defined for the gripper
    '''
    def _drag_arm_impedance(self, gripper_target_pose:bdSE3Pose, spot_target_pose:bdSE3Pose,
                                    reference_frame = BODY_FRAME_NAME, duration_sec=15.0):
        '''Commands the robot arm to perform an impedance command
        which allows it to move heavy objects or perform surface 
        interaction.
        This will force the robot to stand and support the arm. 
        Args:
            pose: The desired end-effector pose.
            reference_frame: The frame in which the pose is defined.
            duration_sec: The max duration given for the command to conclude.
        Returns:
            True if the command succeeded, False otherwise.
        TODO:
            - Try to apply and retain force on the gripper
            - If no object is in hand, raise an error
            - Try work with stiffness
            - Try make the relative pose between the arm and the robot constant
            
        Reference: Spot sdk python examples: arm_impedance_control.py'''

        
        self._log.info(f'Building Impedance Cmd')

        '''
        Part I: Build up the arm impedance control cmd 
        '''
        stand_command = self._get_body_assist_stance_command()
        
        gripper_arm_cmd = robot_command_pb2.RobotCommand()
        gripper_arm_cmd.CopyFrom(stand_command)  # Make sure we keep adjusting the body for the arm
        impedance_cmd = gripper_arm_cmd.synchronized_command.arm_command.arm_impedance_command
        self._log.info("Start building impedance cmd")
        # Set up our root frame; task frame, and tool frame are set by default
        impedance_cmd.root_frame_name = reference_frame

        # Set up stiffness and damping matrices. Note: if these values are set too high,
        # the arm can become unstable. Currently, these are the max stiffness and
        # damping values that can be set.

        # NOTE: Max stiffness: [500, 500, 500, 60, 60, 60]
        #      Max damping: [2.5, 2.5, 2.5, 1.0, 1.0, 1.0]
        impedance_cmd.diagonal_stiffness_matrix.CopyFrom(
            geometry_pb2.Vector(values=[500, 500, 500, 60, 60, 60]))
        impedance_cmd.diagonal_damping_matrix.CopyFrom(
            geometry_pb2.Vector(values=[2.0, 2.0, 2.0, 0.55, 0.55, 0.55]))

        # Set up our `desired_tool` trajectory. This is where we want the tool to be with respect
        # to the task frame. The stiffness we set will drag the tool towards `desired_tool`.
        traj = impedance_cmd.task_tform_desired_tool
        pt1 = traj.points.add()
        pt1.time_since_reference.CopyFrom(seconds_to_duration(5.0))
        pt1.pose.CopyFrom(gripper_target_pose.to_proto())


        self._log.info("Build up the arm impedance command")



        # Set the claw to apply force        
        gripper_arm_cmd = CmdBuilder.claw_gripper_close_command(gripper_arm_cmd) 
        # NOTE: in some places more claw pressure helps. The command below
        #       fails. Need to find alternatives.
        # robot_cmd.gripper_command.claw_gripper_command.maximum_torque = 8

        '''
        Part II: send the trajectory command based on the arm&gripper command
        '''
        '''
         # Execute the impedance command
        cmd_id = self.spot._robot_command_client.robot_command(gripper_arm_cmd)
        succeeded = block_until_arm_arrives(self.spot._robot_command_client, 
                                            cmd_id, self._log,
                                            timeout_sec=duration_sec)
        return succeeded
        '''
        
        pose = spot_target_pose.get_closest_se2_transform()
        self._log.info(f'Sending Robot Command.')
        succeeded, _, id = self.spot.trajectory_cmd(
            goal_x=pose.x, goal_y=pose.y,
            goal_heading=pose.angle,
            cmd_duration=duration_sec, 
            reference_frame=reference_frame,
            blocking=False,
            build_on_command=gripper_arm_cmd
        )
        if succeeded:
            block_for_trajectory_cmd(self.spot._robot_command_client,
                                     cmd_id=id, 
                                     feedback_interval_secs=0.5, 
                                     timeout_sec=duration_sec,
                                     logger=self._log)
        else: 
            self._log.error('Failed to send trajectory command.')
            return False
        return True
    
    def _get_body_assist_stance_command(self, build_on_command=None):
        '''A assistive stance is used when manipulating heavy
        objects or interacting with the environment. 
        
        Returns: A body assist stance command'''
        body_control = spot_command_pb2.BodyControlParams(
            body_assist_for_manipulation=spot_command_pb2.BodyControlParams.
            BodyAssistForManipulation(enable_hip_height_assist=True, enable_body_yaw_assist=False))
        
        
        stand_command = CmdBuilder.synchro_stand_command(
            params=spot_command_pb2.MobilityParams(body_control=body_control))
        return stand_command



    def _joint_mobility_arm_cmd(self, pose, reference_frame):
        '''Command the arm to a certain pose and request that the 
        body follows. This does not expect there to be any contact.'''
        raise NotImplementedError('This is not currently supported by the robot.')


    def _move_heavy_object(self, pose, reference_frame):
        '''Commands the robot to move an object to a desired pose.
        This involves dragging the object near the goal pose, then
        moving the arm to properly position it.
        
        Ideally this would be done using the ArmImpedanceCommand, but
        this is not currently supported by the robot.
        Instead ArmDragCommand has to first be used followed by 
        Arm impedance command.'''
        self._log.info('Moving a heavy object.')
        pose = self._to_bd_se3(pose, reference_frame)

        # Check if pose is farther than some threshold
        # If so, use drag command to move it closer.
        in_body = self.spot._transform_bd_pose(pose, 
                                        self.default_ref_frame, 
                                        BODY_FRAME_NAME)
        body_dist = np.linalg.norm([in_body.x, in_body.y, in_body.z])
        if body_dist > 0.5:
            self._log.info(f'Object is too far away. Dragging it closer.')

            # Define where the object should stand to position the object.
            t = [-1.0, 0.5, 0.0]
            R = bdQuat.from_yaw(-np.pi/4)  # np.pi/2
            stance = pose * bdSE3Pose(x=t[0], y=t[1], z=t[2], rot=R)
            self._drag_arm_to(stance, self.default_ref_frame,
                              duration_sec=30.0)
        self._log.info("Dragging the object to desired pose with impedance cmd")
        # Move arm to adjust pose. 
        self._drag_arm_impedance(pose, self.default_ref_frame,
                                duration_sec=15.0)



    
