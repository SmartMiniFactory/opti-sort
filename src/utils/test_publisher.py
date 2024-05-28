#!/usr/bin/env python3

# Filename:                     test_publisher.py
# Creation Date:                01/06/2022
# Last Revision Date:           01/06/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Up and Running
#
#...........................................Included Libraries and Message Types.........................................
import rospy
from geometry_msgs.msg import Twist
#...........................................End of Included Libraries and Message Types..................................

#......................................................Main Function......................................................
def talker():
    pub = rospy.Publisher('/mattro/cmd_vel', Twist, queue_size = 10)
    rospy.init_node('test_pub_2', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = Twist()
    print("TestPubNode: up and running")
    while not rospy.is_shutdown():

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.01

        pub.publish(msg)
        rate.sleep()
    
    print("Shutting down ...")
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0

    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        print("Node Terminated")
#................................................End of Main Function......................................................... 