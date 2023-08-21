# Available tasks: goto, grasp, drag
string task_name

# Target pose
geometry_msgs/PoseStamped target_pose

# Status can be: running, success, failure
string status