# controller types
In ROS and Gazebo, various types of controllers can be used to control different aspects of robot joints and actuators. These controllers are part of the `ros_control` framework, and they are typically categorized into position controllers, velocity controllers, effort controllers, and more. Here is a list of common controller types you can use in Gazebo:

### Common Controller Types in ROS Control

1. **Joint State Controllers**
   - **`joint_state_controller/JointStateController`**: Publishes the state (position, velocity, effort) of all joints.

2. **Position Controllers**
   - **`position_controllers/JointPositionController`**: Controls the position of a single joint.
   - **`position_controllers/JointGroupPositionController`**: Controls the position of multiple joints.
   - **`position_controllers/GripperActionController`**: Controls the position of a gripper.

3. **Velocity Controllers**
   - **`velocity_controllers/JointVelocityController`**: Controls the velocity of a single joint.
   - **`velocity_controllers/JointGroupVelocityController`**: Controls the velocity of multiple joints.
   - **`velocity_controllers/JointTrajectoryController`**: Controls the trajectory of multiple joints (position, velocity, and optionally acceleration).

4. **Effort Controllers**
   - **`effort_controllers/JointEffortController`**: Controls the effort (torque/force) of a single joint.
   - **`effort_controllers/JointGroupEffortController`**: Controls the effort of multiple joints.
   - **`effort_controllers/JointPositionController`**: Controls the position of a joint by computing and applying the necessary effort using a PID control loop.

5. **Combined Controllers**
   - **`joint_trajectory_controller/JointTrajectoryController`**: Executes trajectories on multiple joints, supporting position, velocity, and acceleration control.

6. **Four Bar Linkage Controllers**
   - **`four_bar_linkage_controllers/FourBarLinkageController`**: Controls four-bar linkage mechanisms.

### Examples of Controller Configuration

Here are some examples of how to configure these controllers in a YAML file:

**Example: `controller.yaml`**
```yaml
controller_manager:
  ros__parameters:
    joint_state_controller:
      type: joint_state_controller/JointStateController
      publish_rate: 50

    joint1_position_controller:
      type: position_controllers/JointPositionController
      joint: joint1
      pid: {p: 100.0, i: 0.01, d: 10.0}

    joint2_velocity_controller:
      type: velocity_controllers/JointVelocityController
      joint: joint2
      pid: {p: 0.1, i: 0.01, d: 0.01}

    joint3_effort_controller:
      type: effort_controllers/JointEffortController
      joint: joint3
      pid: {p: 10.0, i: 0.0, d: 1.0}

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController
      joints:
        - joint1
        - joint2
        - joint3
      constraints:
        goal_time: 0.6
        stopped_velocity_tolerance: 0.02
        joint1: {trajectory: 0.1, goal: 0.1}
        joint2: {trajectory: 0.1, goal: 0.1}
        joint3: {trajectory: 0.1, goal: 0.1}
      state_publish_rate: 25
      action_monitor_rate: 10
```

### Adding Controllers in URDF

When specifying controllers in the URDF, you typically use the `<gazebo>` plugin tag for `ros_control` to load the controllers.

**Example: URDF**
```xml
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/my_robot</robotNamespace>
  </plugin>
</gazebo>
```

### Starting Controllers in Launch File

You can start the controllers using a launch file, where you load the controller configurations and spawn the controllers.

**Example: `my_robot_control.launch`**
```xml
<launch>
  <rosparam file="$(find my_robot_description)/config/controller.yaml" command="load" />
  <node name="controller_spawner" pkg="controller_manager" type="spawner" args="joint_state_controller joint1_position_controller joint2_velocity_controller joint3_effort_controller joint_trajectory_controller" respawn="false" output="screen" />
</launch>
```

By using these controller types and properly configuring them in your YAML and launch files, you can control various aspects of your robot's joints and actuators in Gazebo simulations.