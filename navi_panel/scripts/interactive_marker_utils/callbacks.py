import rospy

class MarkerClickCallback(object):
    def __init__(self, server_name):
        rospy.loginfo("Setting up client...")
        # TODO set up client
        rospy.loginfo(f"Waiting for {server_name} server...")
        # self._client.wait_for_server()
        rospy.loginfo(f"Connected marker to {server_name}.")

    def __call__(self, feedback):
        # rospy.loginfo(feedback)
        rospy.loginfo(f"Waypoint {feedback.marker_name} clicked.")
        # TODO send message to server updating waypoint selection
