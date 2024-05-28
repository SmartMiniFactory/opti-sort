#!/usr/bin/env python3

# Filename:                     pozyx_read.py
# Creation Date:                04/07/2022
# Last Revision Date:           04/07/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       To be tested
# Notes:                        
#
#.............................................About can_read.py.....................................................
# This code is aimed to read the data published by the Pozyx system and combine them in a propaer way, so that the 
# robot_localzation package is able to use them in the Extended Kalman Filter.
#
#
# Inputs [subscribers]: /zyx
#                       
# Outputs [publishers]: /mattro/pozyx
#                  
#...........................................Included Libraries and Message Types.........................................
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
#...........................................End of Included Libraries and Message Types..................................

#.........................................................Global Variables...............................................
header_info = [0,0,0]

quat = np.zeros(4)

position = np.zeros(3)
#.....................................................End of Global Variables............................................
 
#......................................................Callback Functions ...............................................   
def PozyxCallback(msg): # Read data fom Pozyx
    global position
    
    position[0] = msg.position.x
    position[1] = msg.position.y
    position[2] = msg.position.z
#...................................................End of Callback Functions ...........................................

#...................................................User-defined Functions ..............................................
def quaternion_from_euler(roll, pitch, yaw):   # Convert Euler angles to quaternions
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

def publish_pose():
    global header_info, quat, position
    print("PozyxDataNode: up and running")

    # Create a PoseWithCovarianceStamped message
    pose_msg = PoseWithCovarianceStamped()
    seq_int = 0
    
    while not rospy.is_shutdown():

        pose_msg.header.seq = seq_int
        pose_msg.header.frame_id = "world_pozyx"
        pose_msg.header.stamp = rospy.Time.now()

        pose_msg.child_frame_id = "mattro_base_link"       

        pose_msg.pose.pose.position.x = position[0]
        pose_msg.pose.pose.position.y = position[1]
        pose_msg.pose.pose.position.z = position[2]

        quat = quaternion_from_euler(0.0, 0.0, 0.0)

        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.x = quat[1]
        pose_msg.pose.pose.orientation.x = quat[2]
        pose_msg.pose.pose.orientation.x = quat[3]

        pose_msg.pose.covariance = [1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]

        # Publish the ROS message
        pub1.publish(pose_msg)

    print("\n Node terminated")
#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    nodeRate = 10

    try:
        print("Try running node")
        rospy.init_node('imu_read', anonymous=True)
        loop_rate = rospy.Rate(nodeRate)

        # Define ROS publishers and Subscribers
        sub2 = rospy.Subscriber("/zyx", Pose, PozyxCallback)
        pub1 = rospy.Publisher("/mattro/pozyx", PoseWithCovarianceStamped, queue_size = 10)
        
        publish_pose()

    except rospy.ROSInterruptException:
        print("Node terminated")
#................................................End of Main Function......................................................... 