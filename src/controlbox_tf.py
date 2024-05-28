#!/usr/bin/env python3

# Filename:                     controlbox_tf.py
# Creation Date:                01/09/2022
# Last Revision Date:           16/09/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Up and Running
# Notes:                        
#
#.............................................About can_read.py.....................................................
# This code is aimed to publish the fixed tf transformation from the IMU reference frame to the control box
# reference frame.
#
# Outputs [publishers]: tf
#...........................................Included Libraries and Message Types.........................................
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np
#...........................................End of Included Libraries and Message Types..................................
 
#...................................................User-defined Functions ..............................................
def control_box_tf():
    print("Control Box TF broadcasting: up and running")

    while not rospy.is_shutdown():
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "imu_link"
        static_transformStamped.child_frame_id = "control_box_link"

        static_transformStamped.transform.translation.x = +0.249
        static_transformStamped.transform.translation.y = -0.229
        static_transformStamped.transform.translation.z = +0.0325

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    rospy.init_node('control_box_tf_braodcaster')


    try:
        control_box_tf()
        rospy.spin()

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        pass
#................................................End of Main Function......................................................... 