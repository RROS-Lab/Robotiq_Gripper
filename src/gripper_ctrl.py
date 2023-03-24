#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import binascii, sys, rospy
from robotiq_gripper.msg import change_state
from robotiq_gripper.msg import grip_state
from gripper_lib import *
import gripper_publish
#Communication setup
mm.BAUDRATE=115200
mm.BYTESIZE=8
mm.PARITY="N"
mm.STOPBITS=1
mm.TIMEOUT=0.05    

def sys_command(input_word='exit', sys_exit=True):
    while True:
        end_or_not = input("Do you want to {} (y/n)? ".format(input_word))
        if end_or_not.lower() == 'y':
            return True
        elif end_or_not.lower() == 'n' and sys_exit:
            sys.exit()
        elif end_or_not.lower():
            return False


class gripperControl(RobotiqGripper):
    def __init__(self, robot_name, port_name, slaveaddress=9):
        self.attached_to_robot = robot_name
        self.portname = port_name
        super().__init__(port_name, slaveaddress)
        print("Please check that there is nothing in the gripper's clasp")
        sys_command("continue", sys_exit=True)
        self.resetActivate()
        self.reset()
        self.printInfo()
        self.activate()
        self.printInfo()

        self.force = 100
        self.goal_position = 0
        self.position = 0
        self.speed = 100
        self.goTo(self.position, self.speed, self.force)

    def setSpeed(self, speed):
        self.speed=speed

    def setForce(self, force):
        self.force=force
    
    #Finds if object is grasped
    def objectDetected(self):
        return self.graspObject

    #Moves position to correct distance
    def movePos(self, position, force='undefined', speed='undefined'):
        if force=='undefined':
            force = self.force
        if speed=="undefined":
            speed = self.speed

        self.goTo(position, force, speed)
        self.position= self.getPositionCurrent()[0]
        print("{} gripper moving to position: {}, force: {}, speed: {}".format(self.attached_to_robot,position, force, speed))
    
    #Only cares to grasp object at correct distance 
    def justGrasp(self):
        position=255
        initial_force = 0
        self.goTo(position, self.speed, initial_force)
        if self.objectDetected():
            position = self.getPositionCurrent()[0]
        else:
            position = 255
        time.sleep(1)
        print("Current Position: ", position)
        position+=5
        self.position = position
        self.goTo(self.position,self.speed, self.force)

    def __str__(self) -> str:
        return "ROS integrated Robotiq gripper package"
    
#COMs ports to connect gripper to respective robot
robot_comms = {
    #'bkuka': gripperControl('bkuka', "/dev/ttyUSB1"),
    'gkuka': gripperControl('gkuka', "/dev/ttyUSB1"),
    # 's5': gripperControl('s5', "/dev/ttyUSB2")
}

def callback(data):
    
    robot_name = data.robot_name
    grip = robot_comms[robot_name]
    position= data.position_state
    force = data.force_state
    speed = data.speed_state
    fullGrasp = data.justGrasp
    if fullGrasp:
        grip.justGrasp()
    else:
        grip.movePos(position, force, speed)

def listener():
    rospy.Subscriber("moveGripper", change_state, callback) 
         

if __name__ =="__main__":
    #Initialize node
    rospy.init_node('robotiq_gripper', anonymous=True)

    #Set rate for publishing
    r=rospy.Rate(1)

    #Listens for commands on Topic: moveGripper
    listener()

    #Initializes Publisher to Topic: gripper_state
    pub = rospy.Publisher("gripper_state", grip_state, queue_size=1)

    #Generates constant publishing of messages
    while True:        
        for i in robot_comms:
            gripper = robot_comms[i]       
            gripper_publish.updateGripperState(gripper.attached_to_robot, gripper.position, gripper.speed, gripper.force, gripper.graspObject, pub)
        #waits    
        r.sleep()

    rospy.spin()
