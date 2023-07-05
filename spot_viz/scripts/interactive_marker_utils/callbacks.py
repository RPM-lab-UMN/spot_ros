
import rospy
import actionlib
from spot_msgs.msg import TrajectoryAction, TrajectoryGoal
from spot_msgs.msg import GripperAction, GripperGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Duration
from std_srvs.srv import Trigger, TriggerRequest

from scipy.spatial.transform import Rotation as R
import numpy as np


def _get_ros_stamped_pose(position, orientation):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    o = pose.pose.orientation
    o.x, o.y, o.z, o.w = orientation.x, orientation.y, orientation.z, orientation.w
    p = pose.pose.position
    p.x, p.y, p.z = position.x, position.y, position.z
    return pose

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
        rospy.loginfo("Setting up client...")
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
        rospy.loginfo("Setting up client...")
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
        rospy.loginfo("Setting up client...")
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
        rospy.loginfo("Setting up client...")
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


class MultiGraspToggleCallback(object):

    def __init__(self, menu_handler, marker_server, grasp):
        self.menu_handler = menu_handler
        self.marker_server = marker_server
        self.grasp_dictionary = grasp

    def __call__(self, feedback):
        menu_id = feedback.menu_entry_id

        if self.grasp_dictionary['multigrasp_enabled']:
            self.menu_handler.setCheckState(menu_id, self.menu_handler.UNCHECKED)
            self.grasp_dictionary['multigrasp_enabled'] = False
            rospy.loginfo(f"Disabled {self.grasp_dictionary['name']} for multigrasp")

        else:
            self.menu_handler.setCheckState(menu_id, self.menu_handler.CHECKED)
            self.grasp_dictionary['multigrasp_enabled'] = True
            rospy.loginfo(f"Enabled {self.grasp_dictionary['name']} for multigrasp")

        self.menu_handler.reApply(self.marker_server)
        rospy.loginfo("sending update to marker server")
        self.marker_server.applyChanges()


class MultiGraspActionCallback(object):
    def __init__(self, server_name, grasp_list):
        # TODO implement this
        pass

    def __call__(self, feedback):
        # TODO implement this
        pass
