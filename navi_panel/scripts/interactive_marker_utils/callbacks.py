import rospy
import state

glocalization = None

class SetLocalizationCallback(object):
    def __init__(self, marker):
        self._marker = marker

    def __call__(self, feedback):
        rospy.loginfo(f"Changing localization from {state.localization.name} to {self._marker.name}")
        rospy.loginfo(f"global test: {glocalization.name}")
        glocalization = self._marker
        state.localization = self._marker


class NavigateCallback(object):
    def __init__(self):
        pass

    def __call__(self, feedback):
        rospy.loginfo(f"{feedback.marker_name} : NavigateCallback")
