# Robotiq_Gripper 85F 2 Fingers on ROS
Robotiq Gripper ROS package to give fulll functionality of the two finger robotiq gripper on ROS.

- The file name `gripper_ctrl.py` controls gripper.
- The command is based on publishing to topic about: `position(0~255)`, `speed(0~255)`, `force(0~255)` and `justGrap(bool)`.
- To publish gripper state use following command: `rostopic pub /moveGripper robotiq_gripper/change_state robot_name position speed force justGrap`
- Above command can also be executed by the function named `pub_command(robot_name, position, speed=255, force=100, justGrap="False")` from the file `gripper_ctrl.py`.
