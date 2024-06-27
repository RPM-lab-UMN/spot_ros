# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""WASD & Grasp driving of robot."""

from __future__ import print_function
import curses
import io
import logging
import math
import os
import signal
import sys
import threading
import time
from collections import OrderedDict

from PIL import Image, ImageEnhance
from google.protobuf import wrappers_pb2
import numpy as np
import cv2
import apriltag
import matplotlib

import torch
import torchvision
from DINO.collect_dino_features import *
from DINO.dino_wrapper import *

import bosdyn.api.basic_command_pb2 as basic_command_pb2
import bosdyn.api.power_pb2 as PowerServiceProto
# import bosdyn.api.robot_command_pb2 as robot_command_pb2
import bosdyn.api.robot_state_pb2 as robot_state_proto
import bosdyn.api.spot.robot_command_pb2 as spot_command_pb2
import bosdyn.client.util
from bosdyn.api import arm_command_pb2, estop_pb2, geometry_pb2, image_pb2
from bosdyn.api import manipulation_api_pb2,robot_command_pb2, synchronized_command_pb2
from bosdyn.api import network_compute_bridge_service_pb2_grpc
from bosdyn.api import network_compute_bridge_pb2
from bosdyn.client.network_compute_bridge_client import NetworkComputeBridgeClient

from bosdyn.client import ResponseError, RpcError, create_standard_sdk
from bosdyn.client.async_tasks import AsyncGRPCTask, AsyncPeriodicQuery, AsyncTasks
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client import frame_helpers
from bosdyn.client.frame_helpers import ODOM_FRAME_NAME, VISION_FRAME_NAME, BODY_FRAME_NAME, get_vision_tform_body, math_helpers
from bosdyn.client.image import ImageClient
from bosdyn.client.manipulation_api_client import ManipulationApiClient
from bosdyn.client.lease import Error as LeaseBaseError
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.power import PowerClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, block_until_arm_arrives
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.time_sync import TimeSyncError
from bosdyn.util import duration_str, format_metric, secs_to_hms

LOGGER = logging.getLogger()

VELOCITY_BASE_SPEED = 0.5  # m/s
VELOCITY_BASE_ANGULAR = 0.8  # rad/sec
VELOCITY_CMD_DURATION = 0.6  # seconds
COMMAND_INPUT_RATE = 0.1


g_image_click = None
g_image_display = None



def _grpc_or_log(desc, thunk):
    try:
        return thunk()
    except (ResponseError, RpcError) as err:
        LOGGER.error('Failed %s: %s', desc, err)


def _image_to_ascii(image, new_width):
    """Convert an rgb image to an ASCII 'image' that can be displayed in a terminal."""

    ASCII_CHARS = '@#S%?*+;:,.'

    enhancer = ImageEnhance.Contrast(image)
    image = enhancer.enhance(0.8)

    # Scaling image before rotation by 90 deg.
    scaled_rot_height = new_width
    original_rot_width, original_rot_height = image.size
    scaled_rot_width = (original_rot_width * scaled_rot_height) // original_rot_height
    # Scaling rotated width (height, after rotation) by half because ASCII chars
    #  in terminal seem about 2x as tall as wide.
    image = image.resize((scaled_rot_width // 2, scaled_rot_height))

    # Rotate image 90 degrees, then convert to grayscale.
    image = image.transpose(Image.ROTATE_270)
    image = image.convert('L')

    def _pixel_char(pixel_val):
        return ASCII_CHARS[pixel_val * len(ASCII_CHARS) // 256]

    img = []
    row = [' '] * new_width
    last_col = new_width - 1
    for idx, pixel_char in enumerate(_pixel_char(val) for val in image.getdata()):
        idx_row = idx % new_width
        row[idx_row] = pixel_char
        if idx_row == last_col:
            img.append(''.join(row))
    return img

# Helper functions for movements down a trajectory
def block_for_trajectory_cmd(command_client, cmd_id, timeout_sec=None, verbose=False):
    """Helper that blocks until a trajectory command reaches STATUS_AT_GOAL or a timeout is
        exceeded.
       Args:
        command_client: robot command client, used to request feedback
        cmd_id: command ID returned by the robot when the trajectory command was sent
        timeout_sec: optional number of seconds after which we'll return no matter what the
                        robot's state is.
        verbose: if we should print state at 10 Hz.
       Return values:
        True if reaches STATUS_AT_GOAL, False otherwise.
    """
    start_time = time.time()

    if timeout_sec is not None:
        end_time = start_time + timeout_sec
        now = time.time()

    while timeout_sec is None or now < end_time:
        feedback_resp = command_client.robot_command_feedback(cmd_id)

        current_state = feedback_resp.feedback.mobility_feedback.se2_trajectory_feedback.status

        if verbose:
            current_state_str = basic_command_pb2.SE2TrajectoryCommand.Feedback.Status.Name(current_state)

            current_time = time.time()
            print('Walking: ({time:.1f} sec): {state}'.format(
                time=current_time - start_time, state=current_state_str),
                  end='                \r')

        if current_state == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL:
            return True

        time.sleep(0.1)
        now = time.time()

    if verbose:
        print('block_for_trajectory_cmd: timeout exceeded.')
# Helper functions for take_image_grab

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


def add_grasp_constraint(config, grasp, robot_state_client):
    # There are 3 types of constraints:
    #   1. Vector alignment
    #   2. Full rotation
    #   3. Squeeze grasp
    #
    # You can specify more than one if you want and they will be OR'ed together.

    # For these options, we'll use a vector alignment constraint.
    use_vector_constraint = config.force_top_down_grasp or config.force_horizontal_grasp

    # Specify the frame we're using.
    grasp.grasp_params.grasp_params_frame_name = BODY_FRAME_NAME

    if use_vector_constraint:
        if config.force_top_down_grasp:
            # Add a constraint that requests that the x-axis of the gripper is pointing in the
            # negative-z direction in the vision frame.

            # The axis on the gripper is the x-axis.
            axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=1, y=0, z=0)

            # The axis in the vision frame is the negative z-axis
            axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=-1)

        if config.force_horizontal_grasp:
            # Add a constraint that requests that the y-axis of the gripper is pointing in the
            # positive-z direction in the vision frame.  That means that the gripper is constrained to be rolled 90 degrees and pointed at the horizon.

            # The axis on the gripper is the y-axis.            print("Test grasp")

            axis_on_gripper_ewrt_gripper = geometry_pb2.Vec3(x=0, y=1, z=0)

            # The axis in the vision frame is the positive z-axis
            axis_to_align_with_ewrt_vo = geometry_pb2.Vec3(x=0, y=0, z=1)
        if config.force_horizontal_grasp2:
            # Another way to grasp the object horizontally
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


    elif config.force_45_angle_grasp:
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

    elif config.force_0_angle_grasp:
        
        robot_state = robot_state_client.get_robot_state()
        vision_T_body = get_vision_tform_body(robot_state.kinematic_state.transforms_snapshot)

        # Rotation from the body to our desired grasp.
        body_Q_grasp = math_helpers.Quat.from_yaw(0)  # 0 degrees
        vision_Q_grasp = vision_T_body.rotation * body_Q_grasp

        # Turn into a proto
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.rotation_with_tolerance.rotation_ewrt_frame.CopyFrom(body_Q_grasp.to_proto())

        # We'll accept anything within +/- 20 degrees
        constraint.rotation_with_tolerance.threshold_radians = 0.34/4

    elif config.force_squeeze_grasp:
        # Tell the robot to just squeeze on the ground at the given point.
        constraint = grasp.grasp_params.allowable_orientation.add()
        constraint.squeeze_grasp.SetInParent()

def make_robot_command(arm_joint_traj):
    """ Helper function to create a RobotCommand from an ArmJointTrajectory.
        The returned command will be a SynchronizedCommand with an ArmJointMoveCommand
        filled out to follow the passed in trajectory. """

    joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_traj)
    arm_command = arm_command_pb2.ArmCommand.Request(arm_joint_move_command=joint_move_command)
    sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
    arm_sync_robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=sync_arm)
    return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)


## Helper functions for find_people_go
def compute_stand_location_and_yaw(vision_tform_target, robot_state_client,
                                distance_margin):

    # Compute drop-off location:
    #   Draw a line from Spot to the person
    #   Back up 2.0 meters on that line
    vision_tform_robot = frame_helpers.get_a_tform_b(
        robot_state_client.get_robot_state(
        ).kinematic_state.transforms_snapshot, frame_helpers.VISION_FRAME_NAME,
        frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)


    # Compute vector between robot and person
    robot_rt_person_ewrt_vision = [
        vision_tform_robot.x - vision_tform_target.x,
        vision_tform_robot.y - vision_tform_target.y,
        vision_tform_robot.z - vision_tform_target.z
    ]


    # Compute the unit vector.
    if np.linalg.norm(robot_rt_person_ewrt_vision) < 0.01:
        robot_rt_person_ewrt_vision_hat = vision_tform_robot.transform_point(1, 0, 0)
    else:
        robot_rt_person_ewrt_vision_hat = robot_rt_person_ewrt_vision / np.linalg.norm(
            robot_rt_person_ewrt_vision)


    # Starting at the person, back up meters along the unit vector.
    drop_position_rt_vision = [
        vision_tform_target.x +
        robot_rt_person_ewrt_vision_hat[0] * distance_margin,
        vision_tform_target.y +
        robot_rt_person_ewrt_vision_hat[1] * distance_margin,
        vision_tform_target.z +
        robot_rt_person_ewrt_vision_hat[2] * distance_margin
    ]


    # We also want to compute a rotation (yaw) so that we will face the person when dropping.
    # We'll do this by computing a rotation matrix with X along
    #   -robot_rt_person_ewrt_vision_hat (pointing from the robot to the person) and Z straight up:
    xhat = -robot_rt_person_ewrt_vision_hat
    zhat = [0.0, 0.0, 1.0]
    yhat = np.cross(zhat, xhat)
    mat = np.matrix([xhat, yhat, zhat]).transpose()
    heading_rt_vision = math_helpers.Quat.from_matrix(mat).to_yaw()

    return drop_position_rt_vision, heading_rt_vision

def pose_dist(self, pose1, pose2):
    diff_vec = [pose1.x - pose2.x, pose1.y - pose2.y, pose1.z - pose2.z]
    return np.linalg.norm(diff_vec)

def get_walking_params(max_linear_vel, max_rotation_vel):
    max_vel_linear = geometry_pb2.Vec2(x=max_linear_vel, y=max_linear_vel)
    max_vel_se2 = geometry_pb2.SE2Velocity(linear=max_vel_linear,
                                        angular=max_rotation_vel)
    vel_limit = geometry_pb2.SE2VelocityLimit(max_vel=max_vel_se2)
    params = RobotCommandBuilder.mobility_params()
    params.vel_limit.CopyFrom(vel_limit)
    return params

def get_bounding_box_image(response):
    dtype = np.uint8
    img = np.fromstring(response.image_response.shot.image.data, dtype=dtype)
    if response.image_response.shot.image.format == image_pb2.Image.FORMAT_RAW:
        img = img.reshape(response.image_response.shot.image.rows,
                        response.image_response.shot.image.cols)
    else:
        img = cv2.imdecode(img, -1)


    # Draw bounding boxes in the image for all the detections.
    for obj in response.object_in_image:
        conf_msg = wrappers_pb2.FloatValue()
        obj.additional_properties.Unpack(conf_msg)
        confidence = conf_msg.value

        polygon = []
        min_x = float('inf')
        min_y = float('inf')
        for v in obj.image_properties.coordinates.vertexes:
            polygon.append([v.x, v.y])
            min_x = min(min_x, v.x)
            min_y = min(min_y, v.y)

        polygon = np.array(polygon, np.int32)
        polygon = polygon.reshape((-1, 1, 2))
        cv2.polylines(img, [polygon], True, (0, 255, 0), 2)

        caption = "{} {:.3f}".format(obj.name, confidence)
        cv2.putText(img, caption, (int(min_x), int(min_y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return img


class ExitCheck(object):
    """A class to help exiting a loop, also capturing SIGTERM to exit the loop."""

    def __init__(self):
        self._kill_now = False
        signal.signal(signal.SIGTERM, self._sigterm_handler)
        signal.signal(signal.SIGINT, self._sigterm_handler)

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        return False

    def _sigterm_handler(self, _signum, _frame):
        self._kill_now = True

    def request_exit(self):
        """Manually trigger an exit (rather than sigterm/sigint)."""
        self._kill_now = True

    @property
    def kill_now(self):
        """Return the status of the exit checker indicating if it should exit."""
        return self._kill_now


class CursesHandler(logging.Handler):
    """logging handler which puts messages into the curses interface"""

    def __init__(self, fetch_interface):
        super(CursesHandler, self).__init__()
        self._fetch_interface = fetch_interface

    def emit(self, record):
        msg = record.getMessage()
        msg = msg.replace('\n', ' ').replace('\r', '')
        self._fetch_interface.add_message('{:s} {:s}'.format(record.levelname, msg))


class AsyncRobotState(AsyncPeriodicQuery):
    """Grab robot state."""

    def __init__(self, robot_state_client):
        super(AsyncRobotState, self).__init__("robot_state", robot_state_client, LOGGER,
                                              period_sec=0.2)

    def _start_query(self):
        return self._client.get_robot_state_async()
        
  

class AsyncImageCapture(AsyncGRPCTask):
    """Grab camera images from the robot."""

    def __init__(self, robot, config):
        super(AsyncImageCapture, self).__init__()
        self._image_client = robot.ensure_client(ImageClient.default_service_name)
        self._ascii_image = None
        self._video_mode = False
        self._should_take_image = False
        self._image_source = config.image_source

    @property
    def ascii_image(self):
        """Return the latest captured image as ascii."""
        return self._ascii_image

    def toggle_video_mode(self):
        """Toggle whether doing continuous image capture."""
        self._video_mode = not self._video_mode

    def take_image(self):
        """Request a one-shot image."""
        self._should_take_image = True

    def _start_query(self):
        self._should_take_image = False
        source_name = self._image_source
        return self._image_client.get_image_from_sources_async([source_name])

    def _should_query(self, now_sec):  # pylint: disable=unused-argument
        return self._video_mode or self._should_take_image

    def _handle_result(self, result):
        image = result[0]
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

        img_title = "Captured Image"
        cv2.imshow(img_title, img)
        key = cv2.waitKey(5000) & 0xFF
        if key == ord('f') or key == ord('F'):
            # Quit
            print('Saving the current shot.')
            now_secs = time.time()
            cv2.imwrite("./frames/Capture-" + str(now_secs) + ".jpg", img)
        # Close the window if not in video mode
        if not self._video_mode or key == ord('q') or key == ord('Q'):
            cv2.destroyAllWindows()


    def _handle_error(self, exception):
        LOGGER.exception('Failure getting image: %s', exception)
  


class FetchInterface(object):
    """A curses interface for driving the robot."""

    def __init__(self, robot, config):
        self._robot = robot
        # Create clients -- do not use the for communication yet.
        self._lease_client = robot.ensure_client(LeaseClient.default_service_name)
        try:
            self._estop_client = self._robot.ensure_client(EstopClient.default_service_name)
            self._estop_endpoint = EstopEndpoint(self._estop_client, 'GNClient', 9.0)
        except:
            # Not the estop.
            self._estop_client = None
            self._estop_endpoint = None
        self._power_client = robot.ensure_client(PowerClient.default_service_name)
        self._robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        self._robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        self._robot_state_task = AsyncRobotState(self._robot_state_client)

        self._lock = threading.Lock()
        self._locked_messages = ['', '', '']  # string: displayed message for user

        self._config = config

        self._image_task = AsyncImageCapture(robot, config)
        self._async_tasks = AsyncTasks([self._robot_state_task, self._image_task])
        self._command_dictionary = {
            27: self._stop,  # ESC key
            ord('\t'): self._quit_program,
            ord('T'): self._toggle_time_sync,
            ord(' '): self._toggle_estop,
            ord('r'): self._self_right,
            ord('P'): self._toggle_power,
            ord('v'): self._sit,
            ord('b'): self._battery_change_pose,
            ord('f'): self._stand,
            ord('w'): self._move_forward,
            ord('s'): self._move_backward,
            ord('a'): self._strafe_left,
            ord('d'): self._strafe_right,
            ord('q'): self._turn_left,
            ord('e'): self._turn_right,
            ord('I'): self._image_task.take_image,
            ord('O'): self._image_task.toggle_video_mode,

            # New features
            ord('G'): self._take_image_grab,
            # TODO: fix up find people go
            # ord('F'): self._find_people_go,
            ord('D'): self._drop_item,

        
            ord('u'): self._unstow,
            ord('j'): self._stow,
            ord('l'): self._toggle_lease
        }
        
        self._estop_keepalive = None
        self._exit_check = None

        # Stuff that is set in start()
        self._robot_id = None
        self._lease_keepalive = None
        self._hold_sth = False

    def start(self):
        """Begin communication with the robot."""
        # Construct our lease keep-alive object, which begins RetainLease calls in a thread.
        self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                               return_at_exit=True)

        self._robot_id = self._robot.get_id()
        if self._estop_endpoint is not None:
            self._estop_endpoint.force_simple_setup(
            )  # Set this endpoint as the robot's sole estop.

    def shutdown(self):
        """Release control of robot as gracefully as possible."""
        LOGGER.info("Shutting down arm_grasp Interface.")
        if self._estop_keepalive:
            # This stops the check-in thread but does not stop the robot.
            self._estop_keepalive.shutdown()
        if self._lease_keepalive:
            self._lease_keepalive.shutdown()

    def flush_and_estop_buffer(self, stdscr):
        """Manually flush the curses input buffer but trigger any estop requests (space)"""
        key = ''
        while key != -1:
            key = stdscr.getch()
            if key == ord(' '):
                self._toggle_estop()

    def add_message(self, msg_text):
        """Display the given message string to the user in the curses interface."""
        with self._lock:
            self._locked_messages = [msg_text] + self._locked_messages[:-1]

    def message(self, idx):
        """Grab one of the 3 last messages added."""
        with self._lock:
            return self._locked_messages[idx]

    @property
    def robot_state(self):
        """Get latest robot state proto."""
        return self._robot_state_task.proto

    def drive(self, stdscr):
        """User interface to control the robot via the passed-in curses screen interface object."""
        with ExitCheck() as self._exit_check:
            curses_handler = CursesHandler(self)
            curses_handler.setLevel(logging.INFO)
            LOGGER.addHandler(curses_handler)

            stdscr.nodelay(True)  # Don't block for user input.
            stdscr.resize(26, 96)
            stdscr.refresh()

            # for debug
            curses.echo()

            try:
                while not self._exit_check.kill_now:
                    self._async_tasks.update()
                    self._drive_draw(stdscr, self._lease_keepalive)

                    try:
                        cmd = stdscr.getch()
                        # Do not queue up commands on client
                        self.flush_and_estop_buffer(stdscr)
                        self._drive_cmd(cmd)
                        time.sleep(COMMAND_INPUT_RATE)
                    except Exception:
                        # On robot command fault, sit down safely before killing the program.
                        self._safe_power_off()
                        time.sleep(2.0)
                        raise

            finally:
                LOGGER.removeHandler(curses_handler)

    def _drive_draw(self, stdscr, lease_keep_alive):
        """Draw the interface screen at each update."""
        stdscr.clear()  # clear screen
        stdscr.resize(26, 96)
        stdscr.addstr(0, 0, '{:20s} {}'.format(self._robot_id.nickname,
                                               self._robot_id.serial_number))
        stdscr.addstr(1, 0, self._lease_str(lease_keep_alive))
        stdscr.addstr(2, 0, self._battery_str())
        stdscr.addstr(3, 0, self._estop_str())
        stdscr.addstr(4, 0, self._power_state_str())
        stdscr.addstr(5, 0, self._time_sync_str())
        for i in range(3):
            stdscr.addstr(7 + i, 2, self.message(i))
        stdscr.addstr(10, 0, "Commands: [TAB]: quit                               ")
        stdscr.addstr(11, 0, "          [T]: Time-sync, [SPACE]: Estop, [P]: Power")
        stdscr.addstr(12, 0, "          [I]: Take image, [O]: Video mode          ")
        stdscr.addstr(13, 0, "          [f]: Stand, [r]: Self-right               ")
        stdscr.addstr(14, 0, "          [v]: Sit, [b]: Battery-change             ")
        stdscr.addstr(15, 0, "          [wasd]: Directional strafing              ")
        stdscr.addstr(16, 0, "          [qe]: Turning, [ESC]: Stop              ")
        stdscr.addstr(17, 0, "          [l]: Return/Acquire lease                 ")
        stdscr.addstr(18, 0, "          == New Features ==                          ")
        stdscr.addstr(19, 0, "          [G]: Find a good point in the image and grasp")
        stdscr.addstr(20, 0, "          [D]: Release the object and Stow the arm")
        stdscr.addstr(21, 0, "")

        

        stdscr.refresh()


    def _drive_cmd(self, key):
        """Run user commands at each update."""
        try:
            cmd_function = self._command_dictionary[key]
            cmd_function()

        except KeyError:
            if key and key != -1 and key < 256:
                self.add_message("Unrecognized keyboard command: '{}'".format(chr(key)))

    def _try_grpc(self, desc, thunk):
        try:
            return thunk()
        except (ResponseError, RpcError, LeaseBaseError) as err:
            self.add_message("Failed {}: {}".format(desc, err))
            return None

    def _try_grpc_async(self, desc, thunk):

        def on_future_done(fut):
            try:
                fut.result()
            except (ResponseError, RpcError, LeaseBaseError) as err:
                self.add_message("Failed {}: {}".format(desc, err))
                return None

        future = thunk()
        future.add_done_callback(on_future_done)

    def _quit_program(self):
        self._sit()
        if self._exit_check is not None:
            self._exit_check.request_exit()

    def _toggle_time_sync(self):
        if self._robot.time_sync.stopped:
            self._robot.start_time_sync()
        else:
            self._robot.time_sync.stop()

    def _toggle_estop(self):
        """toggle estop on/off. Initial state is ON"""
        if self._estop_client is not None and self._estop_endpoint is not None:
            if not self._estop_keepalive:
                self._estop_keepalive = EstopKeepAlive(self._estop_endpoint)
            else:
                self._try_grpc("stopping estop", self._estop_keepalive.stop)
                self._estop_keepalive.shutdown()
                self._estop_keepalive = None

    def _toggle_lease(self):
        """toggle lease acquisition. Initial state is acquired"""
        if self._lease_client is not None:
            if self._lease_keepalive is None:
                self._lease_keepalive = LeaseKeepAlive(self._lease_client, must_acquire=True,
                                                       return_at_exit=True)
            else:
                self._lease_keepalive.shutdown()
                self._lease_keepalive = None

    def _start_robot_command(self, desc, command_proto, end_time_secs=None):

        def _start_command():
            self._robot_command_client.robot_command(command=command_proto,
                                                     end_time_secs=end_time_secs)

        self._try_grpc(desc, _start_command)

    def _self_right(self):
        self._start_robot_command('self_right', RobotCommandBuilder.selfright_command())

    def _battery_change_pose(self):
        # Default HINT_RIGHT, maybe add option to choose direction?
        self._start_robot_command(
            'battery_change_pose',
            RobotCommandBuilder.battery_change_pose_command(
                dir_hint=basic_command_pb2.BatteryChangePoseCommand.Request.HINT_RIGHT))

    def _sit(self):
        self._start_robot_command('sit', RobotCommandBuilder.synchro_sit_command())

    def _stand(self):
        self._start_robot_command('stand', RobotCommandBuilder.synchro_stand_command())

    def _move_forward(self):
        self._velocity_cmd_helper('move_forward', v_x=VELOCITY_BASE_SPEED)

    def _move_backward(self):
        self._velocity_cmd_helper('move_backward', v_x=-VELOCITY_BASE_SPEED)

    def _strafe_left(self):
        self._velocity_cmd_helper('strafe_left', v_y=VELOCITY_BASE_SPEED)

    def _strafe_right(self):
        self._velocity_cmd_helper('strafe_right', v_y=-VELOCITY_BASE_SPEED)

    def _turn_left(self):
        self._velocity_cmd_helper('turn_left', v_rot=VELOCITY_BASE_ANGULAR)

    def _turn_right(self):
        self._velocity_cmd_helper('turn_right', v_rot=-VELOCITY_BASE_ANGULAR)

    def _stop(self):
        self._start_robot_command('stop', RobotCommandBuilder.stop_command())

    def _velocity_cmd_helper(self, desc='', v_x=0.0, v_y=0.0, v_rot=0.0):
        self._start_robot_command(
            desc, RobotCommandBuilder.synchro_velocity_command(v_x=v_x, v_y=v_y, v_rot=v_rot),
            end_time_secs=time.time() + VELOCITY_CMD_DURATION)
    
        
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
        self.add_message('Click on an object to start grasping...')
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
                self.add_message('"q" pressed, exiting.')
                exit(0)

        self.add_message('Picking object at image location (' + str(g_image_click[0]) + ', ' +
                          str(g_image_click[1]) + ')')

        pick_vec = geometry_pb2.Vec2(x=g_image_click[0], y=g_image_click[1])
        return pick_vec
    

    def _get_pick_vec_apriltag(self, image):
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
        self.add_message('Find the position to grasp using AprilTag')
        image_title = 'To grasp using AprilTag'
        cv2.namedWindow(image_title)
        # define the AprilTags detector options and then detect the AprilTags
        # in the input image
        self.add_message("[INFO] detecting AprilTags...")
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(img)
        self.add_message("[INFO] {} total AprilTags detected".format(len(results)))

        if len(results)>1:
            self.add_message("Which object should I choose...?")
            assert False
        
       
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = results[0].corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(img, ptA, ptB, (0, 255, 0), 2)
        cv2.line(img, ptB, ptC, (0, 255, 0), 2)
        cv2.line(img, ptC, ptD, (0, 255, 0), 2)
        cv2.line(img, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(results[0].center[0]), int(results[0].center[1]))
        cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)  
        # draw the tag family on the image
        tagFamily = results[0].tag_family.decode("utf-8")
        cv2.putText(img, tagFamily, (ptA[0], ptA[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        self.add_message("[INFO] tag family: {}".format(tagFamily))

        # show the output image after AprilTag detection
        cv2.imshow(image_title, img)
        cv2.waitKey(0)
        

        self.add_message('Picking object at image location (' + str(cX) + ', ' +
                          str(cY) + ')')

        pick_vec = geometry_pb2.Vec2(x=cX, y=cY)
        return pick_vec
    
    def _get_pick_vec_DINO(self, image):

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
        
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        torch.backends.cudnn.benchmark = True

        # Hard-code the default cfg for building DINO model
        cfg = {}
        cfg['dino_strides'] = 4
        cfg['desired_height'] = img.shape[0]
        cfg['desired_width'] = img.shape[1]
        cfg['use_16bit'] = False
        cfg['use_traced_model'] = False
        cfg['cpu'] = False
        cfg['similarity_thresh'] = 0.1

        model = get_dino_pixel_wise_features_model(cfg = cfg, device = device)


        img_feat = preprocess_frame(img, cfg=cfg)
        img_feat = model(img_feat)
        
        img_feat_norm = torch.nn.functional.normalize(img_feat, dim=1)


        img_feat_eval = torch.load("./queries/feat1.pt")
        img_feat_eval = img_feat_eval[0].view(1,-1)
        img_feat_eval = img_feat_eval.cuda()


        cosine_similarity = torch.nn.CosineSimilarity(dim=1)  # (1, 512, H // 2, W // 2)

        similarity = cosine_similarity(
            img_feat_norm, img_feat_eval.view(1, -1, 1, 1)
        )
        # Viz thresholded "relative" attention scores
        similarity = (similarity + 1.0) / 2.0  # scale from [-1, 1] to [0, 1]
        
        # A strange bug here... the similarity is flipped!!
        similarity = 1 - similarity
        # similarity = similarity.clamp(0., 1.)
        similarity_rel = (similarity - similarity.min()) / (
            similarity.max() - similarity.min() + 1e-12
        )
        similarity_rel = similarity_rel[0]  # 1, H // 2, W // 2 -> # H // 2, W // 2
        similarity_rel[similarity_rel < cfg['similarity_thresh'] ]= 0.0

        similarity_rel = similarity_rel.detach().cpu().numpy()

        similarity_argmin = np.argmin(similarity_rel)

        pick_y = int(similarity_argmin/(similarity_rel.shape[1]))
        pick_x = int(similarity_argmin%(similarity_rel.shape[1]))

        print([pick_x, pick_y])
        cmap = matplotlib.cm.get_cmap("jet")
        similarity_colormap = cmap(similarity_rel)[..., :3]

        img_to_viz = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        _overlay = img_to_viz.astype(np.float32) / 255
        _overlay = 0.5 * _overlay + 0.5 * similarity_colormap
        
        cv2.circle(_overlay, (pick_x, pick_y), 5, (0, 0, 255), -1) 
        cv2.imshow("Debug image for DINO feature method", _overlay)
        cv2.waitKey(0)


        return geometry_pb2.Vec2(x=pick_x, y=pick_y)


    def _command_raise_obj(self, command_client):
        sh0 = 0.0692
        sh1 = -1.882
        el0 = 1.652
        el1 = -0.0691
        wr0 = 1.622
        wr1 = 1.550
        max_vel = wrappers_pb2.DoubleValue(value=1)
        max_acc = wrappers_pb2.DoubleValue(value=5)
        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1, time_since_reference_secs=1.5)
        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point],
                                                            maximum_velocity=max_vel,
                                                            maximum_acceleration=max_acc)
        # Make a RobotCommand
        command = make_robot_command(arm_joint_traj)

        # Send the request
        cmd_id = command_client.robot_command(command)
        self.add_message('Requesting the robot to raise the object')
            

    def _take_image_grab(self):
        # If the robot is already holding something, exit
        if self._hold_sth:
            self.add_message("The robot is already holding something!!!")
            return
        
        # Set up the necessary clients
        robot = self._robot

        assert robot.has_arm(), "Robot requires an arm to run this example."

        # TODO: Command the robot to stand up at first if it is not standing


        robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
        image_client = robot.ensure_client(ImageClient.default_service_name)

        manipulation_api_client = robot.ensure_client(ManipulationApiClient.default_service_name)

        # Take a picture with a camera
        self.add_message('Getting an image from: ' + self._config.image_source)
        image_responses = image_client.get_image_from_sources([self._config.image_source])

        if len(image_responses) != 1:
            self.add_message('Got invalid number of images: ' + str(len(image_responses)))
            self.add_message(image_responses)
            assert False

        image = image_responses[0]
        pick_vec = self._get_pick_vec(image) #Manually pick the grasp point
        #pick_vec = self._get_pick_vec_apriltag(image) #Pick the grasp point using AprilTag
        #pick_vec = self._get_pick_vec_DINO(image) #Use the pre-stored DINO features
        

        # Build the proto
        grasp = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=pick_vec, transforms_snapshot_for_camera=image.shot.transforms_snapshot,
            frame_name_image_sensor=image.shot.frame_name_image_sensor,
            camera_model=image.source.pinhole)
        if self._config.force_0_angle_grasp:
            self.add_message("Try the grasp with rotation constraint!")
        # Optionally add a grasp constraint.  This lets you tell the robot you only want top-down grasps or side-on grasps.
        add_grasp_constraint(self._config, grasp, robot_state_client)

        # Ask the robot to pick up the object
        grasp_request = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=grasp)

        # Send the request
        cmd_response = manipulation_api_client.manipulation_api_command(
            manipulation_api_request=grasp_request)

        attempt_times = 0
        # Get feedback from the robot
        while True:
            feedback_request = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_response.manipulation_cmd_id)

            # Send the request
            response = manipulation_api_client.manipulation_api_feedback_command(
                manipulation_api_feedback_request=feedback_request)

            #self.add_message('Current state: '+ manipulation_api_pb2.ManipulationFeedbackState.Name(response.current_state))


            if response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED or response.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                break
            
            attempt_times = attempt_times + 1
            if attempt_times > 40:
                break
            time.sleep(0.25)

        
        self.add_message('Finished grasp.')
        if self._config.raise_object:
            
            self.add_message('Hoist the object')
            # Raise the object
            command_client = robot.ensure_client(RobotCommandClient.default_service_name)
            self._command_raise_obj(command_client)

            carry_cmd = RobotCommandBuilder.arm_carry_command()
            command_client.robot_command(carry_cmd)

        self._hold_sth = True
        cv2.destroyAllWindows()
    
    
    def get_obj_and_img(self, network_compute_client, server, model_name, confidence, label):

        # Build a network compute request for this image source.
        image_source_and_service = network_compute_bridge_pb2.ImageSourceAndService(
            image_source=self._config.image_source)

        # Input data:
        #   model name
        #   minimum confidence (between 0 and 1)
        #   if we should automatically rotate the image
        input_data = network_compute_bridge_pb2.NetworkComputeInputData(
            image_source_and_service=image_source_and_service,
            model_name=model_name,
            min_confidence=confidence,
            rotate_image=network_compute_bridge_pb2.NetworkComputeInputData.
            ROTATE_IMAGE_ALIGN_HORIZONTAL)

        # Server data: the service name
        server_data = network_compute_bridge_pb2.NetworkComputeServerConfiguration(
            service_name=server)

        # Pack and send the request.
        process_img_req = network_compute_bridge_pb2.NetworkComputeRequest(
            input_data=input_data, server_config=server_data)

        try:
            resp = network_compute_client.network_compute_bridge_command(process_img_req)
        except:
            # This sometimes happens if the NCB is unreachable due to intermittent wifi failures.
            self.add_message('Error connecting to network compute bridge. This may be temporary.')
            return None, None, None

        best_obj = None
        highest_conf = 0.0
        best_vision_tform_obj = None

        img = get_bounding_box_image(resp)
        image_full = resp.image_response

        # Show the image
        cv2.imshow("Fetch", img)
        cv2.waitKey(10000)
        cv2.destroyAllWindows()


        if len(resp.object_in_image) > 0:
            for obj in resp.object_in_image:
                # Get the label
                obj_label = obj.name.split('_label_')[-1]
                if obj_label != label:
                    continue
                conf_msg = wrappers_pb2.FloatValue()
                obj.additional_properties.Unpack(conf_msg)
                conf = conf_msg.value

                try:
                    self.add_message("Check it!")
                    self.add_message(obj.image_properties.frame_name_image_coordinates)
                    vision_tform_obj = frame_helpers.get_a_tform_b(
                        obj.transforms_snapshot,
                        frame_helpers.VISION_FRAME_NAME,
                        obj.image_properties.frame_name_image_coordinates)
                except bosdyn.client.frame_helpers.ValidateFrameTreeError:
                    # No depth data available.
                    vision_tform_obj = None

                if conf > highest_conf and vision_tform_obj is not None:
                    highest_conf = conf
                    best_obj = obj
                    best_vision_tform_obj = vision_tform_obj

        if best_obj is not None:
            return best_obj, image_full, best_vision_tform_obj

        return None, None, None

 

    def _find_people_go(self):
        
        #Set up the clients
        network_compute_client = self._robot.ensure_client(
            NetworkComputeBridgeClient.default_service_name)
        
        robot_state_client = self._robot.ensure_client(
            RobotStateClient.default_service_name)
        
        command_client = self._robot.ensure_client(
            RobotCommandClient.default_service_name)
        

        # Find a person to deliver the toy to
        person, image, vision_tform_person = self.get_obj_and_img(
                network_compute_client, self._config.ml_service,
                'Darknet', 0.6,
                'chair') # Could be any object

        
        if not person:
            # If the camera finds nobody
            self.add_message("No peWrson found! Exit..")
            return
        
        # We now have found a person to drop the toy off near.
        drop_position_rt_vision, heading_rt_vision = compute_stand_location_and_yaw(
                vision_tform_person, robot_state_client, distance_margin=2.0)

        wait_position_rt_vision, wait_heading_rt_vision = compute_stand_location_and_yaw(
                vision_tform_person, robot_state_client, distance_margin=2.5)
        

        # Tell the robot to go there

        # Limit the speed so we don't charge at the person.
        move_cmd = RobotCommandBuilder.trajectory_command(
                goal_x=drop_position_rt_vision[0],
                goal_y=drop_position_rt_vision[1],
                goal_heading=heading_rt_vision,
                frame_name=frame_helpers.VISION_FRAME_NAME,
                params=get_walking_params(0.5, 0.5))
        
        end_time = 5.0
        cmd_id = command_client.robot_command(command=move_cmd,
                                                end_time_secs=time.time() +
                                                end_time)
       
        # Wait until the robot reports that it is at the goal.
        block_for_trajectory_cmd(command_client, cmd_id, timeout_sec=5, verbose=True)

        print('Arrived at goal, dropping object...')

        '''
        # Do an arm-move to gently put the object down.
        # Build a position to move the arm to (in meters, relative to and expressed in the gravity aligned body frame).
        x = 0.75
        y = 0
        z = -0.25
        hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)

        # Point the hand straight down with a quaternion.
        qw = 0.707
        qx = 0
        qy = 0.707
        qz = 0
        flat_body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

        flat_body_tform_hand = geometry_pb2.SE3Pose(
            position=hand_ewrt_flat_body, rotation=flat_body_Q_hand)

        robot_state = robot_state_client.get_robot_state()
        vision_tform_flat_body = frame_helpers.get_a_tform_b(
            robot_state.kinematic_state.transforms_snapshot,
            frame_helpers.VISION_FRAME_NAME,
            frame_helpers.GRAV_ALIGNED_BODY_FRAME_NAME)

        vision_tform_hand_at_drop = vision_tform_flat_body * math_helpers.SE3Pose.from_obj(
            flat_body_tform_hand)

        # duration in seconds
        seconds = 1

        arm_command = RobotCommandBuilder.arm_pose_command(
            vision_tform_hand_at_drop.x, vision_tform_hand_at_drop.y,
            vision_tform_hand_at_drop.z, vision_tform_hand_at_drop.rot.w,
            vision_tform_hand_at_drop.rot.x, vision_tform_hand_at_drop.rot.y,
            vision_tform_hand_at_drop.rot.z, frame_helpers.VISION_FRAME_NAME,
            seconds)

        # Keep the gripper closed.
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(
            0.0)

        # Combine the arm and gripper commands into one RobotCommand
        command = RobotCommandBuilder.build_synchro_command(
            gripper_command, arm_command)

        # Send the request
        cmd_id = command_client.robot_command(command)
        # Wait until the arm arrives at the goal.
        block_until_arm_arrives(command_client, cmd_id)

        # Open the gripper
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(
            1.0)
        command = RobotCommandBuilder.build_synchro_command(gripper_command)
        cmd_id = command_client.robot_command(command)

        # Wait for the dogtoy to fall out
        time.sleep(1.5)

        # Stow the arm.
        stow_cmd = RobotCommandBuilder.arm_stow_command()
        command_client.robot_command(stow_cmd)

        time.sleep(1)
        '''
    def _drop_item(self):
        #TODO: drop the item
        self.add_message("Dropping the item")
        self._hold_sth = False

        global g_image_click, g_image_display
        g_image_click = None
        g_image_display = None
        command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        # Open the gripper
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(
            1.0)
        command = RobotCommandBuilder.build_synchro_command(gripper_command)
        cmd_id = command_client.robot_command(command)

        # Wait for the dogtoy to fall out
        time.sleep(1.5)

        # Stow the arm.
        stow_cmd = RobotCommandBuilder.arm_stow_command()
        command_client.robot_command(stow_cmd)

        time.sleep(1)


    def _stow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_stow_command())

    def _unstow(self):
        self._start_robot_command('stow', RobotCommandBuilder.arm_ready_command())

    def _return_to_origin(self):
        self._start_robot_command(
            'fwd_and_rotate',
            RobotCommandBuilder.synchro_se2_trajectory_point_command(
                goal_x=0.0, goal_y=0.0, goal_heading=0.0, frame_name=ODOM_FRAME_NAME, params=None,
                body_height=0.0, locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT),
            end_time_secs=time.time() + 20)


    def _toggle_power(self):
        power_state = self._power_state()
        if power_state is None:
            self.add_message('Could not toggle power because power state is unknown')
            return

        if power_state == robot_state_proto.PowerState.STATE_OFF:
            self._try_grpc_async("powering-on", self._request_power_on)
        else:
            self._try_grpc("powering-off", self._safe_power_off)

    def _request_power_on(self):
        request = PowerServiceProto.PowerCommandRequest.REQUEST_ON
        return self._power_client.power_command_async(request)

    def _safe_power_off(self):
        self._start_robot_command('safe_power_off', RobotCommandBuilder.safe_power_off_command())

    def _power_state(self):
        state = self.robot_state
        if not state:
            return None
        return state.power_state.motor_power_state

    def _lease_str(self, lease_keep_alive):
        if lease_keep_alive is None:
            alive = 'STOPPED'
            lease = 'RETURNED'
        else:
            try:
                _lease = lease_keep_alive.lease_wallet.get_lease()
                lease = '{}:{}'.format(_lease.lease_proto.resource, _lease.lease_proto.sequence)
            except bosdyn.client.lease.Error:
                lease = '...'
            if lease_keep_alive.is_alive():
                alive = 'RUNNING'
            else:
                alive = 'STOPPED'
        return 'Lease {} THREAD:{}'.format(lease, alive)

    def _power_state_str(self):
        power_state = self._power_state()
        if power_state is None:
            return ''
        state_str = robot_state_proto.PowerState.MotorPowerState.Name(power_state)
        return 'Power: {}'.format(state_str[6:])  # get rid of STATE_ prefix

    def _estop_str(self):
        if not self._estop_client:
            thread_status = 'NOT ESTOP'
        else:
            thread_status = 'RUNNING' if self._estop_keepalive else 'STOPPED'
        estop_status = '??'
        state = self.robot_state
        if state:
            for estop_state in state.estop_states:
                if estop_state.type == estop_state.TYPE_SOFTWARE:
                    estop_status = estop_state.State.Name(estop_state.state)[6:]  # s/STATE_//
                    break
        return 'Estop {} (thread: {})'.format(estop_status, thread_status)

    def _time_sync_str(self):
        if not self._robot.time_sync:
            return 'Time sync: (none)'
        if self._robot.time_sync.stopped:
            status = 'STOPPED'
            exception = self._robot.time_sync.thread_exception
            if exception:
                status = '{} Exception: {}'.format(status, exception)
        else:
            status = 'RUNNING'
        try:
            skew = self._robot.time_sync.get_robot_clock_skew()
            if skew:
                skew_str = 'offset={}'.format(duration_str(skew))
            else:
                skew_str = "(Skew undetermined)"
        except (TimeSyncError, RpcError) as err:
            skew_str = '({})'.format(err)
        return 'Time sync: {} {}'.format(status, skew_str)

    def _battery_str(self):
        if not self.robot_state:
            return ''
        battery_state = self.robot_state.battery_states[0]
        status = battery_state.Status.Name(battery_state.status)
        status = status[7:]  # get rid of STATUS_ prefix
        if battery_state.charge_percentage.value:
            bar_len = int(battery_state.charge_percentage.value) // 10
            bat_bar = '|{}{}|'.format('=' * bar_len, ' ' * (10 - bar_len))
        else:
            bat_bar = ''
        time_left = ''
        if battery_state.estimated_runtime:
            time_left = ' ({})'.format(secs_to_hms(battery_state.estimated_runtime.seconds))
        return 'Battery: {}{}{}'.format(status, bat_bar, time_left)


def _setup_logging(verbose):
    """Log to file at debug level, and log to console at INFO or DEBUG (if verbose).

    Returns the stream/console logger so that it can be removed when in curses mode.
    """
    LOGGER.setLevel(logging.DEBUG)
    log_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')

    # Save log messages to file arm_grasp_log for later debugging.
    file_handler = logging.FileHandler('arm_grasp.log')
    file_handler.setLevel(logging.DEBUG)
    file_handler.setFormatter(log_formatter)
    LOGGER.addHandler(file_handler)

    # The stream handler is useful before and after the application is in curses-mode.
    if verbose:
        stream_level = logging.DEBUG
    else:
        stream_level = logging.INFO

    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(stream_level)
    stream_handler.setFormatter(log_formatter)
    LOGGER.addHandler(stream_handler)
    return stream_handler


def main(argv):
    """Command-line interface."""
    import argparse

    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('-i', '--image-source', help='Get image from source',
                        default='hand_color_image')

    parser.add_argument('-ml', '--ml-service', help='Get server name for machine learning service',
                        default='fetch-server')
    
    parser.add_argument('-t', '--force-top-down-grasp',
                        help='Force the robot to use a top-down grasp (vector_alignment demo)',
                        action='store_true')
    parser.add_argument('-f', '--force-horizontal-grasp',
                        help='Force the robot to use a horizontal grasp (vector_alignment demo)',
                        action='store_true')
    
    parser.add_argument('-f2', '--force-horizontal-grasp2',
                        help='Force the robot to use a horizontal grasp, but in another way',
                        action='store_true')
    parser.add_argument(
        '-r', '--force-45-angle-grasp',
        help='Force the robot to use a 45 degree angled down grasp (rotation_with_tolerance demo)',
        action='store_true')
    parser.add_argument(
        '-r2', '--force-0-angle-grasp',
        help='Force the robot to use a 0 degree angled along yaw grasp (rotation_with_tolerance demo)',
        action='store_true')
    
    parser.add_argument('-ro', '--raise_object', help='Let the robot to raise the object after grasping',
        action='store_true')
    parser.add_argument('-s', '--force-squeeze-grasp',
                        help='Force the robot to use a squeeze grasp', action='store_true')
    
    parser.add_argument('--time-sync-interval-sec',
                        help='The interval (seconds) that time-sync estimate should be updated.',
                        type=float)
    options = parser.parse_args(argv)

    num = 0
    if options.force_top_down_grasp:
        num += 1
    if options.force_horizontal_grasp:
        num += 1
    if options.force_45_angle_grasp:
        num += 1
    if options.force_squeeze_grasp:
        num += 1

    if num > 1:
        print("Error: cannot force more than one type of grasp.  Choose only one.")
        sys.exit(1)


    stream_handler = _setup_logging(options.verbose)

    # Create robot object.
    sdk = create_standard_sdk('FetchClient')
    robot = sdk.create_robot(options.hostname)
    try:
        bosdyn.client.util.authenticate(robot)
        robot.start_time_sync(options.time_sync_interval_sec)
    except RpcError as err:
        LOGGER.error("Failed to communicate with robot: %s" % err)
        return False

    fetch_interface = FetchInterface(robot, options)
    try:
        fetch_interface.start()
    except (ResponseError, RpcError) as err:
        LOGGER.error("Failed to initialize robot communication: %s" % err)
        return False

    LOGGER.removeHandler(stream_handler)  # Don't use stream handler in curses mode.

    try:
        try:
            # Prevent curses from introducing a 1 second delay for ESC key
            os.environ.setdefault('ESCDELAY', '0')
            # Run wasd interface in curses mode, then restore terminal config.
            curses.wrapper(fetch_interface.drive)
        finally:
            # Restore stream handler to show any exceptions or final messages.
            LOGGER.addHandler(stream_handler)
    except Exception as e:
        LOGGER.error("WASD has thrown an error: [%r] %s", e, e)
    finally:
        # Do any final cleanup steps.
        fetch_interface.shutdown()

    return True


if __name__ == "__main__":
    #rospy.init_node("arm_grasp_advanced", anonymous=True)
    if not main(sys.argv[1:]):
        os._exit(1)
    os._exit(0)
