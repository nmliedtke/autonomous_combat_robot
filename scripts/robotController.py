#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from autonomous_combat_robot.msg import RobotDrive
from autonomous_combat_robot.msg import TwoDOdom
from simple_pid import PID

loops = 0

MAX_OUTPUT = 255
MAX_VEL = 100
MAX_OMEGA = 20

MIDDLE_OUTPUT = 128

set_position = TwoDOdom()
set_position.x = 10
set_position.y = 20
set_position.theta = 180.0
set_position.velocity = 0.0
set_position.omega = 1.0

odom = TwoDOdom()
odom.x = 0
odom.y = 0
odom.theta = 0.0
odom.velocity = 0.0
odom.omega = 0.0

control_state = 'VELOCITY' # possible states IDLE, SETPOINT, VELOCITY
setpoint_state = 'IDLE' # possbile states IDLE, TURN, MOVE

loops = 0

def updateSetPosition(data):
    print('update set position')
    set_position = data

def updateOdom(data):
    print('update odom')
    odom = data

def updateControlMode(data):
    print('update control mode')
    control_state = data

def setpointLoops(drivePub):
    print('setpoint loops')

def velocityLoops(drivePub):
    print('velocity loops')
    output = int((set_position.velocity / MAX_VEL) * (MAX_OUTPUT - MIDDLE_OUTPUT) + MIDDLE_OUTPUT)

    delta = int((set_position.omega / MAX_OMEGA) * (MAX_OUTPUT - MIDDLE_OUTPUT) + MIDDLE_OUTPUT)
    outputR = output + delta
    outputL = output - delta
    
def start_controller():
    rospy.init_node("robot_controller", anonymous=True)
    drivePub = rospy.Publisher("/robot_drive", RobotDrive, queue_size=10)
    rospy.Subscriber("/set_position", TwoDOdom, updateSetPosition)
    rospy.Subscriber("/odom", TwoDOdom, updateOdom)
    rospy.Subscriber("/control_mode", String, updateControlMode)
    global loops
    while not rospy.is_shutdown():
        if control_state == 'IDLE':
            loops = loops + 1
        if control_state == 'SETPOINT':
            loops = loops + 1
            setpointLoops(drivePub)
        if control_state == 'VELOCITY':
            loops = loops + 1
            velocityLoops(drivePub)


if __name__ == '__main__':
    loops = 0
    start_controller()
