import rospy
#import actionlib
#from spot_msgs.msg import NavigateToAction, NavigateToGoal

class SetLocalizationCallback(object):
    def __init__(self, int_marker, manager):
        self._marker = int_marker
        self._manager = manager

    def __call__(self, feedback):
        self._manager.update_localization(self._marker)


class NavigateCallback(object):
    def __init__(self, manager):
        self._manager = manager
        # self._client = actionlib.SimpleActionClient(manager, NavigateToAction)

    def __call__(self, feedback):
        rospy.loginfo(f"{feedback.marker_name} : NavigateCallback, localization={self._manager.get_localization()}")
        # goal = NavigateToGoal(feedback.name, False, )
