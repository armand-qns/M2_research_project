# M2 Research project

This package implements a Cartesian velocity controller for ROS 2.

## Prerequisites

This package requires the **Pinocchio** library for kinematic and dynamic calculations.

Please ensure that Pinocchio is installed and properly configured in your environment before building or running this package.

## Usage

Once the controller is running, you can send velocity commands (Twist) via the dedicated topic.

### Command Example

To publish a single message and test the `my_cartesian_velocity_controller`, use the following command in your terminal:

```bash
ros2 topic pub --once /my_cartesian_velocity_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
