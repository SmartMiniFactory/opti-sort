#!/usr/bin/env python3

# Filename:                     test_rotate.py
# Creation Date:                12/08/2022
# Last Revision Date:           12/08/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Up and Ready
# Notes:                        

#.............................................About can_read.py.....................................................
# This code is aimed to make the mattro rovo 2 rotate of roughly 90 degrees using orientation data from the IMU
#
#
# Inputs [subscribers]: /filter/quaternion
#                       
# Outputs [publishers]: /mattro/cmd_vel
#                  
#...........................................Included Libraries and Message Types.........................................
import rospy
import numpy as np
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Twist
#...........................................End of Included Libraries and Message Types..................................

#.........................................................Global Variables...............................................
quat = np.zeros(4)
#.....................................................End of Global Variables............................................

#.........................................................Global Constants...............................................
ANGULAR_ROTATION = 90 # [deg]
#.....................................................End of Global Constants............................................

#......................................................Callback Functions ...............................................   
def ImuCallback(msg): # Read data from the IMU
    global quat

    quat[0] = msg.quaternion.x
    quat[1] = msg.quaternion.y
    quat[2] = msg.quaternion.z
    quat[3] = msg.quaternion.w
#...................................................End of Callback Functions ...........................................

#...................................................User-defined Functions ..............................................
def Quaternion2Euler(quat):

    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    euler = np.array([X, Y, Z])
    
    return euler

def mattro_rotate(loop_rate):
    global header_info, quat
    print("SquarePathNode: up and running")

    # Create a Twist message
    cmd_vel_msg = Twist()

    state = 0
    starting_heading = 0.0
    current_heading = 0.0
    
    while not rospy.is_shutdown():
        
        euler = Quaternion2Euler(quat)

        if state == 0:

            starting_heading = euler[2]
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = 0.0

            if starting_heading == 0.0:
                state = 0
            elif starting_heading != 0.0:
                state = 1
            
        if state == 1:
            current_heading = euler[2]
            rotation = np.abs(current_heading - starting_heading)
            print("Starting Yaw: ", starting_heading, "Current Yaw: ", current_heading, "DeltaYaw: ", rotation, end = "\r")    
            if rotation >= ANGULAR_ROTATION:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.linear.z = 0.0

                cmd_vel_msg.angular.x = 0.0
                cmd_vel_msg.angular.y = 0.0
                cmd_vel_msg.angular.z = 0.0

                state = 2

            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = 0.3

        if state == 2:
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = 0.0

           
        # Publish the ROS message
        pub1.publish(cmd_vel_msg)
        loop_rate.sleep()
    
    print("Shutting down ...")
    print("\n Node terminated")
#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    nodeRate = 10

    try:
        print("Try running node")
        rospy.init_node('test_orientation', anonymous=True)
        loop_rate = rospy.Rate(nodeRate)

        # Define ROS publishers and Subscribers
        sub1 = rospy.Subscriber("/filter/quaternion", QuaternionStamped, ImuCallback)
        pub1 = rospy.Publisher("/mattro/cmd_vel", Twist, queue_size = 10)
        
        mattro_rotate(loop_rate)

    except rospy.ROSInterruptException:
        print("Node terminated")
#................................................End of Main Function......................................................... 