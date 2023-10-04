import rospy
import actionlib
from spot_msgs.msg import NavigateToAction, NavigateToGoal

class SetLocalizationCallback(object):
    def __init__(self, int_marker, manager):
        self._marker = int_marker
        self._manager = manager

    def __call__(self, feedback):
        self._manager.update_localization(self._marker)


class NavigateCallback(object):
    def __init__(self, manager):
        self._manager = manager
        self._client = actionlib.SimpleActionClient("spot/navigate_to", NavigateToAction)

    def __call__(self, feedback):
        rospy.loginfo(f"{feedback.marker_name} : NavigateCallback, localization={self._manager.get_localization()}")
        if self._manager.get_localization() == None:
            rospy.loginfo("No localization waypoint specified")
            return
        rospy.loginfo(dir(self._manager.get_localization()))
        goal = NavigateToGoal(feedback.marker_name, False, self._manager.get_localization().name)
        self._client.send_goal(goal)
        self._client.wait_for_result()
        result = self._client.get_result()
        rospy.loginfo(result)
        #self._manager.clear_localization()
