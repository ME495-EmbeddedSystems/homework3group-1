# ME495 Embedded Systems Homework 3 Part 2 - moveit_wrapper
Authors: Stephen Ferro, Anuj Natraj, Jihai Zhao, Carter DiOrio, Kyle Wang
Description: A package to simplify using the Moveit package with robot arms. The package includes a class which can be used to plan and execute trajectories. 
1. Intialize a MoveItApi() object with the following configuration options:
    - node: the node used to interface with ROS
    - base_frame: the base frame of the robot being controlled
    - end_effector_frame: the frame of the end effector of the robot being controlled 
    - group_name: the name of the action group
    - joint_state_topic: the topic where joint states are published
2. Use the plan() function within the MoveItApi() object to plan and optionally execute a path
    - max_velocity_scaling_factor: sets the maximum velocity scaling factor (default: 0.1)
    - max_acceleration_scaling_factor: sets the maximum velocity scaling factor (default: 0.1)
    - point: Desired end point of the end effector (default: {None})
    - orientation (geometry_msgs/Quaternion) -- Desired orientation of the end effector (default: {None})
    - start_pose (geometry_msgs/Pose) -- The starting pose of the robot (default: {Current Pose})
    - execute (bool) -- Execute the trajectory (default: {False})
    - use_jc (bool) -- Use joint constraints instead of goal constraints (default: {True})
