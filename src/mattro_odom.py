#!/usr/bin/env python3

# Filename:                     mattro_odom.py
# Creation Date:                16/09/2022
# Last Revision Date:           17/09/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Up and Running
# Notes:                        
#
#.............................................About can_read.py.....................................................
# This code is aimed to compute a simple odometry based on the feeback from the motors' encoders.
#
#
# Inputs [subscribers]: /mattro/bock_status
#                       
# Outputs [publishers]: /mattro/odometry
#                  
#...........................................Included Libraries and Message Types.........................................
import rospy
import numpy as np
from bock_controller.msg import BockStatus
from nav_msgs.msg import Odometry
#...........................................End of Included Libraries and Message Types..................................

#.........................................................Global Variables...............................................
v_r = 0.0
v_l = 0.0
#.....................................................End of Global Variables............................................

#......................................................Callback Functions ...............................................   
def SpeedCallback(msg): # Read the pose of the mattro
    global v_r, v_l

    v_l = msg.speed_left
    v_r = msg.speed_right
#...................................................End of Callback Functions ...........................................

#...................................................User-defined Functions ..............................................
def quaternion_from_euler(roll, pitch, yaw):  # Convert Euler angles to quaternions
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def compute_odometry(loop_rate, wheel_space):
    global v_r, v_l
    print("MattroOdomNode: up and running")

    odomMattro = Odometry()
    seq_int = 0

    X = 0.0
    Y = 0.0
    theta = 0.0
    dt = 0.1 # [sec]

    while not rospy.is_shutdown():
        v = (v_l + v_r)/2
        omega = (v_r - v_l)/wheel_space

        X = X + v * np.cos(theta) * dt
        Y = Y + v * np.sin(theta) * dt
        theta = theta + omega * dt
        
        odomMattro.header.seq = seq_int
        odomMattro.header.frame_id = "world"
        odomMattro.header.stamp = rospy.Time.now()

        odomMattro.child_frame_id = "mattro_base_link"
        
        odomMattro.pose.pose.position.x = X + v * np.cos(theta) * dt
        odomMattro.pose.pose.position.y = Y + v * np.sin(theta) * dt
        odomMattro.pose.pose.position.z = 0.600

        quat = quaternion_from_euler(0.0, 0.0, theta + omega * dt)
        odomMattro.pose.pose.orientation.x = quat[0]
        odomMattro.pose.pose.orientation.y = quat[1]
        odomMattro.pose.pose.orientation.z = quat[2]
        odomMattro.pose.pose.orientation.w = quat[3]

        odomMattro.pose.covariance = [1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]

        odomMattro.twist.twist.linear.x = v
        odomMattro.twist.twist.linear.y = 0.0
        odomMattro.twist.twist.linear.z = 0.0
        odomMattro.twist.twist.angular.x = 0.0
        odomMattro.twist.twist.angular.y = 0.0
        odomMattro.twist.twist.angular.z = omega
        
        odomMattro.twist.covariance = [1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]

        pub1.publish(odomMattro)

        seq_int = seq_int + 1

        loop_rate.sleep()

    # Publish the ROS message
    print("Shutting down ...")
    print("\n Node terminated")
#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    nodeRate = 10

    try:
        print("Try running node")
        rospy.init_node('mattro_odom', anonymous=True)

        # Read the parameters from the launch file
        rate = rospy.get_param("~rate", 10)
        loop_rate = rospy.Rate(rate)
        wheel_space = rospy.get_param("~base_width", 0.644)

        # Define ROS publishers and Subscribers
        sub1 = rospy.Subscriber("/mattro/bock_status", BockStatus, SpeedCallback)
        pub1 = rospy.Publisher("/mattro/odom", Odometry, queue_size = 10)
        
        compute_odometry(loop_rate, wheel_space)

    except rospy.ROSInterruptException:
        print("Node terminated")
#................................................End of Main Function......................................................... 