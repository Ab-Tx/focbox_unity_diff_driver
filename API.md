# Overview

Controller for differential drive wheel systems based on the FOCBOX Unity (VESC based dual motor controller). Control is in the form of a velocity command, that is split then sent on the two wheels. Odometry is computed from the feedback from the hardware, and published.

## Velocity commands

The controller works with a velocity twist from which it extracts the x component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

## Hardware interface type

The controller works with wheel joints through a velocity interface, is is implemented and not exposed, user should not need to mess with it.

If you require two velocity joint interfaces instead of the differential controller, you should modify the controler node arguments in the launch file, coments in said file will help you.

## Other features

    Automatic Handbrake Apllication when stoped
    Realtime-safe implementation.
    Odometry publishing
    Task-space velocity, acceleration and jerk limits
    Automatic stop after command time-out

# Hardware compatibility

Developed for the FOCBOX Unity hardware, should work with derivatives with minor changes.

You should use my modified firmware based on 43.60 as it includes a funcion for reseting the tachometer, this may be required in some applications.

It will work with he original firmware (tested under 43.60) but it will not reset the tachometer when starting the node.


# ROS API

## Description

The controller main input is a geometry_msgs::Twist topic in the namespace of the controller (focbox_unity_diff_driver/diff_controller/).

### Subscribed Topics
cmd_vel (geometry_msgs/Twist)

    Velocity command.

### Published Topics
odom (nav_msgs/Odometry)

    Odometry computed from the hardware feedback.

/tf (tf/tfMessage)

    Transform from odom to base_footprint

publish_cmd (geometry_msgs/TwistStamped)

    Available when "publish_cmd" parameter is set to True. It is the Twist after limiters have been applied on the controller input.

### Driver Parameters
counts (double)

    number of counts of the tachometer per turn

pole_pairs (double)

    number of pole pairs of the motors

### Differential controller parameters
left_wheel (string | string[...])

    Left wheel joint name or list of joint names

right_wheel (string | string[...])

    Right wheel joint name or list of joint names

pose_covariance_diagonal (double[6])

    Diagonal of the covariance matrix for odometry pose publishing

twist_covariance_diagonal (double[6])

    Diagonal of the covariance matrix for odometry twist publishing

publish_rate (double, default: 50.0)

    Frequency (in Hz) at which the odometry is published. Used for both tf and odom

wheel_separation_multiplier (double, default: 1.0)

    Multiplier applied to the wheel separation parameter. This is used to account for a difference between the robot model and a real robot.

wheel_radius_multiplier (double, default: 1.0)

    Multiplier applied to the wheel radius parameter. This is used to account for a difference between the robot model and a real robot.

cmd_vel_timeout (double, default: 0.5)

    Allowed period (in s) allowed between two successive velocity commands. After this delay, a zero speed command will be sent to the wheels.

base_frame_id (string, default: base_link)

    Base frame_id, which is used to fill in the child_frame_id of the Odometry messages and TF.

linear/x/has_velocity_limits (bool, default: false)

    Whether the controller should limit linear speed or not.

linear/x/max_velocity (double)

    Maximum linear velocity (in m/s)

linear/x/min_velocity (double)

    Minimum linear velocity (in m/s). Setting this to 0.0 will disable backwards motion. When unspecified, -max_velocity is used.

linear/x/has_acceleration_limits (bool, default: false)

    Whether the controller should limit linear acceleration or not.

linear/x/max_acceleration (double)

    Maximum linear acceleration (in m/s^2)

linear/x/min_acceleration (double)

    Minimum linear acceleration (in m/s^2). When unspecified, -max_acceleration is used.

linear/x/has_jerk_limits (bool, default: false)

    Whether the controller should limit linear jerk or not.

linear/x/max_jerk (double)

    Maximum linear jerk (in m/s^3).

angular/z/has_velocity_limits (bool, default: false)

    Whether the controller should limit angular velocity or not.

angular/z/max_velocity (double)

    Maximum angular velocity (in rad/s)

angular/z/min_velocity (double)

    Minimum angular velocity (in rad/s). Setting this to 0.0 will disable counter-clockwise rotation. When unspecified, -max_velocity is used.

angular/z/has_acceleration_limits (bool, default: false)

    Whether the controller should limit angular acceleration or not.

angular/z/max_acceleration (double)

    Maximum angular acceleration (in rad/s^2)

angular/z/min_acceleration (double)

    Minimum angular acceleration (in rad/s^2). When unspecified, -max_acceleration is used.

angular/z/has_jerk_limits (bool, default: false)

    Whether the controller should limit angular jerk or not.

angular/z/max_jerk (double)

    Maximum angular jerk (in m/s^3).

enable_odom_tf (bool, default: true)

    Publish to TF directly or not

wheel_separation (double)

    The distance of the left and right wheel(s). The diff_drive_controller will attempt to read the value from the URDF if this parameter is not specified

wheel_radius (double)

    Radius of the wheels. It is expected they all have the same size. The diff_drive_controller will attempt to read the value from the URDF if this parameter is not specified.

odom_frame_id (string, default: "/odom")

    Name of frame to publish odometry in.

publish_cmd (bool, default: False)

    Publish the velocity command to be executed. It is to monitor the effect of limiters on the controller input.

allow_multiple_cmd_vel_publishers (bool, default: False)

    Setting this to true will allow more than one publisher on the input topic, ~/cmd_vel. Setting this to false will cause the controller to brake if there is more than one publisher on ~/cmd_vel.

# Controller configuration examples

## Complete description
```
focbox_unity_driver:
  # parameters used in the driver
  motor_parameters:
    counts: 90.0
    pole_pairs: 14.0

  # joint state interface
  joint_states:
    type: joint_state_controller/JointStateController
    publish_rate: 50

  # velocity joint interface (only if diff_drive_controller is not used)
  velocity_controller1:
    type: velocity_controllers/JointVelocityController
    joint: motor1

  # velocity joint interface (only if diff_drive_controller is not used)
  velocity_controller2:
    type: velocity_controllers/JointVelocityController
    joint: motor2

  # diff_drive_controller (ros_control) parameters
  differential_controller:
    type        : "diff_drive_controller/DiffDriveController"
    left_wheel  : 'motor2'
    right_wheel : 'motor1'
    publish_rate: 50.0               # default: 50
    pose_covariance_diagonal : [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]
    twist_covariance_diagonal: [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1000.0]

    # Wheel separation and diameter. These are both optional.
    # diff_drive_controller will attempt to read either one or both from the
    # URDF if not specified as a parameter
    wheel_separation : 0.465
    wheel_radius : 0.0875

    # Wheel separation and radius multipliers
    wheel_separation_multiplier: 1.0 # default: 1.0
    wheel_radius_multiplier    : 1.0 # default: 1.0

    # Velocity commands timeout [s], default 0.5
    cmd_vel_timeout: 10

    # Base frame_id
    base_frame_id: base_footprint #default: base_link

    # Velocity and acceleration limits
    # Whenever a min_* is unspecified, default to -max_*
    linear:
      x:
        has_velocity_limits    : false
        max_velocity           : 1.0  # m/s
        min_velocity           : -0.5 # m/s
        has_acceleration_limits: false
        max_acceleration       : 0.8  # m/s^2
        min_acceleration       : -0.4 # m/s^2
        has_jerk_limits        : false
        max_jerk               : 5.0  # m/s^3
    angular:
      z:
        has_velocity_limits    : false
        max_velocity           : 1.7  # rad/s
        has_acceleration_limits: false
        max_acceleration       : 1.5  # rad/s^2
        has_jerk_limits        : false
        max_jerk               : 2.5  # rad/s^3
```

## Launch file
```
<!-- -*- mode: XML -*- -->
<launch>

  <!-- Parameters for the hardware interface and controllers -->
  <rosparam file="$(find focbox_unity_driver)/config/focbox_unity_driver_config.yaml"/>

  <!-- Start a controller for our joints -->
  <node name="joint_controllers_spawner" pkg="controller_manager" type="spawner"
          respawn="false" output="screen"
          args="/focbox_unity_driver/joint_states
                /focbox_unity_driver/differential_controller" />
  <!-- change "/focbox_unity_driver/differential_controller" for 
  "/focbox_unity_driver/velocity_controller1
  /focbox_unity_driver/velocity_controller2" 
  this exports a velocity state interface for each wheel instead of the diff controller-->

  <!--FOCBOX Unitydriver parameters (/dev/focbox is a symlink created so there is no need to modify this launch file) -->
  <arg name="port" default="/dev/focbox" />

  <!--FOCBOX Unitydriver node -->
  <node pkg="focbox_unity_driver" type="focbox_unity_driver_node" name="focbox_unity_driver_node"
        output="screen" required="true">
    <param name="port" value="$(arg port)" />
  </node>

</launch>

```