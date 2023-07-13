
import rospy
import actionlib
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal
from spot_msgs.msg import GripperAction, GripperGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Duration
from std_srvs.srv import Trigger, TriggerRequest

from scipy.spatial.transform import Rotation as R
import numpy as np

from typing import final


def _get_ros_stamped_pose(position, orientation):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    o = pose.pose.orientation
    o.x, o.y, o.z, o.w = orientation.x, orientation.y, orientation.z, orientation.w
    p = pose.pose.position
    p.x, p.y, p.z = position.x, position.y, position.z
    return pose


class CheckBoxCallback(object):
    """
    See wiki page on GitHub for more detailed instructions.


    Builder class for the callback for a checkbox button in rviz.
    Handles all the boilerplate for enabling/disabling the "enabled"
    state of the checkbox & sending the update to the menu server--
    all that needs to be added by a subclass is the code to be executed
    when the box is enabled, when it is disabled, and code to be run
    whenever the button is pressed regardless of status, both before
    AND after the on-enable/on-disable code.

    The methods _before_either, _after_either, on_enable, and on_disable
    MUST be implemented by a subclass, even if all they do is pass
    """
    def __init__(self, menu_handler, marker_server):
        """
        :param menu_handler: The MenuHandler object inserting the button using this callback
        :param marker_server: The InteractiveMarkerServer that this button pertains to
        """
        self.menu_handler = menu_handler
        self.marker_server = marker_server

    def _before_either(self):
        raise NotImplementedError("Must be implemented by a subclass")

    def _on_enable(self):
        raise NotImplementedError("Must be implemented by a subclass")

    def _on_disable(self):
        raise NotImplementedError("Must be implemented by a subclass")

    def _after_either(self):
        raise NotImplementedError("Must be implemented by a subclass")

    @final
    def __call__(self, feedback):
        entry_id = feedback.menu_entry_id
        check_state = self.menu_handler.getCheckState(entry_id)

        self._before_either()

        if check_state == self.menu_handler.UNCHECKED:
            rospy.logdebug(f"Enabling checkbox button with id {entry_id}")
            self.menu_handler.setCheckState(entry_id, self.menu_handler.CHECKED)
            self._on_enable()
        else:
            rospy.logdebug(f"Disabling checkbox button with id {entry_id}")
            self.menu_handler.setCheckState(entry_id, self.menu_handler.UNCHECKED)

        self._after_either()

        rospy.logdebug(f"Applying changes for checkbox button with id {entry_id}")
        rospy.logdebug("sending update to menu server")
        self.marker_server.applyChanges()

class TriggerCallback():
    def __init__(self, topic_name):
        rospy.loginfo(f'Connecting to {topic_name}.')
        self._srvs = rospy.ServiceProxy(topic_name, Trigger)
        rospy.loginfo("Waiting for service...")
        self._srvs.wait_for_service()
        rospy.loginfo("Connected! ")
    def __call__(self, feedback):
        result = self._srvs(TriggerRequest())
        rospy.loginfo(result)

class GoToMarkerCallback(object):
    '''
    Modified to accomodate motion to position based on input 1x3 vectors t and R
    containing translation and rotation with respect to detection of ficudial marker.

    Args:
        server_name (str): name of the action server to connect to.
        t (list): the offset of the grasp relative to the marker center.
        R (list): the rotation of the grasp relative to the marker orientation.
    '''

    def __init__(self, server_name, t=[0.0, 0.0, 0.0], R=[0, 0, 0]):
        rospy.loginfo(f"Setting up client for {server_name}...")
        self._client = actionlib.SimpleActionClient(server_name, TrajectoryAction)
        rospy.loginfo(f"Waiting for {server_name} server...")
        self._client.wait_for_server()
        rospy.loginfo(f"Connected ChairHandle marker to {server_name}.")
        self._t, self._R = t, R

    def __call__(self, feedback):
        # rospy.loginfo(feedback)
        rospy.loginfo("Sending navigation goal to server...")
        ros_pose = _get_ros_stamped_pose(feedback.pose.position, feedback.pose.orientation)
        ros_pose.header.frame_id = feedback.header.frame_id
        duration = Duration()
        duration.data.secs = 5
        goal = TrajectoryGoal(target_pose=ros_pose, duration=duration)
        goal.target_pose.pose = _get_perpendicular_pose(goal.target_pose.pose,
                                            rot_vec=self._R,
                                            offset=self._t)
        self._client.send_goal(goal)

        self._client.wait_for_result()
        result = self._client.get_result()
        rospy.loginfo(result)

class GoToRightMarkerCallback(object):

    def __init__(self, server_name, t=[0.2, 0.0, 0.5], R=[0, 0, 0]):
        '''
        A functor providing callback that triggers a rasp action server.
        Args:
            server_name (str): name of the action server to connect to.
            t (list): the offset of the grasp relative to the marker center.
            R (list): the rotation of the grasp relative to the marker orientation.'''
        rospy.loginfo(f"Setting up client for {server_name}...")
        self._client = actionlib.SimpleActionClient(server_name, TrajectoryAction)
        rospy.loginfo(f"Waiting for {server_name} server...")
        self._client.wait_for_server()
        rospy.loginfo(f"Connected ChairHandle marker to {server_name}.")
        self._t, self._R = t, R

    def __call__(self, feedback):
        rospy.loginfo("Sending grasp goal to server...")
        ros_pose = _get_ros_stamped_pose(feedback.pose.position, feedback.pose.orientation)
        ros_pose.header.frame_id = feedback.header.frame_id
        goal = TrajectoryGoal(pose=ros_pose.pose, header=ros_pose.header)
        goal.pose = _get_perpendicular_pose(goal.pose,
                                            rot_vec=self._R,
                                            offset=self._t)
        self._client.send_goal(goal, feedback_cb=rospy.loginfo)
        self._client.wait_for_result()
        result = self._client.get_result()
        rospy.loginfo(result)

def _get_perpendicular_pose(pose, rot_vec=[0, 0, np.pi/2], offset=[0.0, 0.0, 0.0]):
    rot = R.from_rotvec(rot_vec)
    quat = pose.orientation
    pose_rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    pose_quat = (pose_rot * rot).as_quat()
    quat.x, quat.y, quat.z, quat.w = pose_quat

    print(f'offset: {offset}')
    offset = pose_rot.as_matrix() @ np.array(offset).T
    print(f'pose_rot.as_matrix(): {pose_rot.as_matrix()}')
    p = pose.position
    p.x += offset[0]; p.y += offset[1]; p.z += offset[2]
    pose.position = p
    print(pose.position)
    return pose

class GrabMarkerCallback(object):

    def __init__(self, server_name, grasp_pos, t=[0.2, 0.0, 0.5], R=[0, 0, 0]):
        '''
        A functor providing callback that triggers a rasp action server.
        Args:
            server_name (str): name of the action server to connect to.
            t (list): the offset of the grasp relative to the marker center.
            R (list): the rotation of the grasp relative to the marker orientation.'''
        rospy.loginfo(f"Setting up client for {server_name}...")
        self._client = actionlib.SimpleActionClient(server_name, GripperAction)
        rospy.loginfo(f"Waiting for {server_name} server...")
        self._client.wait_for_server()
        rospy.loginfo(f"Connected ChairHandle marker to {server_name}.")
        self._t, self._R = t, R
        self._grasp_pos = grasp_pos

    def __call__(self, feedback):
        rospy.loginfo("Sending grasp goal to server...")
        ros_pose = _get_ros_stamped_pose(feedback.pose.position, feedback.pose.orientation)
        ros_pose.header.frame_id = feedback.header.frame_id
        goal = GripperGoal(pose=ros_pose.pose, header=ros_pose.header)
        goal.pose = _get_perpendicular_pose(goal.pose,
                                            rot_vec=self._R,
                                            offset=self._t)
        self._client.send_goal(goal, feedback_cb=rospy.loginfo)
        self._client.wait_for_result()
        self._grasp_pos['t'] = self._t
        self._grasp_pos['R'] = self._R
        result = self._client.get_result()
        rospy.loginfo(result)

class DragToMarkerCallback(object):

    def __init__(self, server_name, grasp_pos, t=[0.0, 0.0, 0.0], R=[0, 0, 0]):
        rospy.loginfo(f"Setting up client for {server_name}...")
        self._client = actionlib.SimpleActionClient(server_name, GripperAction)
        rospy.loginfo(f"Waiting for {server_name} server...")
        self._client.wait_for_server()
        rospy.loginfo(f"Connected ChairHandle marker to {server_name}.")
        self._t, self._R = t, R
        self._grasp_pos = grasp_pos

    def __call__(self, feedback):
        if self._grasp_pos['t'] == None or self._grasp_pos['R'] == None:
            return
        rospy.loginfo("Sending manipulation goal...")
        ros_pose = _get_ros_stamped_pose(feedback.pose.position, feedback.pose.orientation)
        ros_pose.header.frame_id = feedback.header.frame_id
        goal = GripperGoal(pose=ros_pose.pose, header=ros_pose.header)
        goal.pose = _get_perpendicular_pose(goal.pose,
                                            rot_vec=self._grasp_pos['t'],
                                            offset=self._grasp_pos['R'])
        self._client.send_goal(goal, feedback_cb=rospy.loginfo)
        self._client.wait_for_result()
        result = self._client.get_result()
        rospy.loginfo(result)

class TrackingToggleCallback(CheckBoxCallback):
    def __init__(self, menu_handler, marker_server, marker_pose_sub):
        super().__init__(menu_handler, marker_server)
        self._pose_update_subscriber = marker_pose_sub

    def _before_either(self):
        pass

    def _on_enable(self):
        pass

    def _on_disable(self):
        pass

    def _after_either(self):
        self._pose_update_subscriber.toggle()
        rospy.logdebug("Toggled chair marker pose topic subscription")

class MultiGraspToggleCallback(CheckBoxCallback):
    def __init__(self, menu_handler, marker_server, grasp):
        super().__init__(menu_handler, marker_server)
        self.grasp_dictionary = grasp

    def _before_either(self):
        pass

    def _on_enable(self):
        self.grasp_dictionary['multigrasp_enabled'] = True

    def _on_disable(self):
        self.grasp_dictionary['multigrasp_enabled'] = False

    def _after_either(self):
        pass

class MultiGraspActionCallback(object):
    def __init__(self, server_name, grasp_pos, grasp_points):
        """
        A function that provides a callback which takes in a list of possible grasp
        targets, chooses which one it deems "best", then executes that grasp
        Args:
            server_name (str): name of the ROS action server to contact
            grasp_pos (obj): stores the current grasp position. written/read by callbacks
            grasp_points (dict[]): list of grasp points in the form of dictionaries
        """
        self.grasp_pos = grasp_pos
        self.grasps = grasp_points

    def __call__(self, feedback):
        rospy.loginfo("multigrasp callback - enabled grasps:")
        for grasp in self.grasps:
            if grasp['multigrasp_enabled']:
                rospy.loginfo(grasp)
