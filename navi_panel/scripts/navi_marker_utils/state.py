import rospy

# Color values for markers
SELECTED_R = 0.05
SELECTED_G = 0.45
SELECTED_B = 0.98
UNSELECTED_R = 0.45
UNSELECTED_G = 0.06
UNSELECTED_B = 0.67


class StateManager(object):
    def __init__(self, marker_server):
        self._server = marker_server
        self._localization = None

    def update_localization(self, int_marker):
        if self._localization is None:
            rospy.loginfo(f"Setting localization to {int_marker.name}")

        else:
            rospy.loginfo(f"Changing localization from {self._localization.name} to {int_marker.name}")

            self._localization.controls[0].markers[0].color.r = UNSELECTED_R
            self._localization.controls[0].markers[0].color.g = UNSELECTED_G
            self._localization.controls[0].markers[0].color.b = UNSELECTED_B

        self._localization = int_marker

        self._localization.controls[0].markers[0].color.r = SELECTED_R
        self._localization.controls[0].markers[0].color.g = SELECTED_G
        self._localization.controls[0].markers[0].color.b = SELECTED_B

        self._server.applyChanges()

    def get_localization(self):
        return self._localization

    def clear_localization(self):
        self._localization = None
