import rospy


class SetLocalizationCallback(object):
    def __init__(self):
        pass

    def __call__(self, feedback):
        rospy.loginfo(f"Using {feedback.marker_name} as localization")
        localization = feedback.marker_name


class NavigateCallback(object):
    def __init__(self):
        pass

    def __call__(self, feedback):
        rospy.loginfo(f"{feedback.marker_name} : NavigateCallback")
