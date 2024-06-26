Date: 09/15/2023
Author: Xun Tu

This directory, called "ros_wrapper", should contain the definitions, 
as well as the behaviors, of the ROS-related classes particularly designed for SPOT, 
such as servers, publishers, clients, subscribers, etc. 
For example, if I want to operate the gripper using ROS, 
I should define the associated ROS action server/Clients
in this directory. The ROS servers/clients will handle the communications 
within ROS platform, such as publishing messages to other nodes
Also, they will call functions defined in spot_task_wrapper or spot_wrapper 
to complete the desired tasks. 