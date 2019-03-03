#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from autonomous_combat_robot.msg import RobotDrive
import serial
import struct

ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(1)

def callback(data):
    lm = data.motorLeft
    rm = data.motorRight
    w = data.weapon
    print("lm: " + str(lm) + 'rm: ' + str(rm) + 'w: ' + str(w))
    ser.write(struct.pack('>BBB',lm, rm, w))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('printSerial', anonymous=True)

    rospy.Subscriber("/robot_drive", RobotDrive, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
