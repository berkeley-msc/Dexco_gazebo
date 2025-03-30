# requirements

1. numpy 1.x seems needed for building the workspace
2. follow up the installation in the package, universal_robot, to setup the package, the raw github is https://github.com/ros-industrial/universal_robot.
3. verify the succeed installation of ur by running the ur_gazebo.

# run
1. roslaunch ur_gazebo ur5_bringup_jd.launch
2. roslaunch ur5_moveit_config moveit_rviz.launch
3. roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
In this way you can control the ur5 using moveit interface in rviz and control the gripper with a gui.