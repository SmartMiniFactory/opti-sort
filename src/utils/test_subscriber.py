#!/usr/bin/env python3
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

def TwistCallback(data):
    dx = data.linear.x
    dr = data.angular.z
    print(dx, dr)

def listener():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/mattro/cmd_vel", Twist, TwistCallback)
    rospy.spin()

if __name__ == '__main__':
    listener()