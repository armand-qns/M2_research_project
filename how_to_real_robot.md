TERMINAL 1

ros2 launch franka_bringup franka.launch.py     robot_ip:=172.16.2.2


TERMINAL 2

ros2 control load_controller my_cartesian_velocity_controller
ros2 control set_controller_state my_cartesian_velocity_controller inactive
ros2 control switch_controllers --activate my_cartesian_velocity_controller^C
ros2 topic pub --once /my_cartesian_velocity_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
