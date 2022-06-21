#! /usr/bin/env python3
from std_msgs.msg import Int32, String
from robotiq_gripper.msg import grip_state
import rospy, sys


    #Updates Gripper State topic 
def updateGripperState(robot_name, position, speed, force, activated):
    robot_name = robot_name
    position = position
    force = force
    speed = speed
    activated = activated

    pub_msg = grip_state()

    pub_msg.gripper_name = str(robot_name)
    pub_msg.position_state = int(position)
    pub_msg.force_state = int(force)
    pub_msg.speed_state = int(speed)
    pub_msg.activated = bool(activated)

    r=rospy.Rate(10)
    # rospy.loginfo(pub_msg)
    pub = rospy.Publisher("gripper_state", grip_state, queue_size=10)
    pub.publish(pub_msg)
# def sys_command(input_word='exit', sys_exit=True):
#     while True:
#         end_or_not = input("Do you want to {} (y/n)? ".format(input_word))
#         if end_or_not.lower() == 'y' and sys_exit:
#             sys.exit()
#         elif end_or_not.lower():
#             return True
#         elif end_or_not.lower() == 'n':
#             return False

# while True:
#     gripper_to_robot = input("Which robot are you using: ").lower()
#     if gripper_to_robot in robot_comms:
#         com_port = robot_comms[gripper_to_robot]
#         break
#     else:
#         print("Robot not found")
#         sys_command()

# if __name__ == '__main__':
#     try:
#         updateGripperState()
#     except rospy.ROSInterruptException: pass

