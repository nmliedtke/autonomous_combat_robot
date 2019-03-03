#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from autonomous_combat_robot.msg import RobotDrive

def talker():
    pub = rospy.Publisher('/robot_drive', RobotDrive, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        odom = RobotDrive()
        odom.motorLeft = 10
        odom.motorRight = 15
        odom.weapon = 200

        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(odom)
        pub.publish(odom)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
