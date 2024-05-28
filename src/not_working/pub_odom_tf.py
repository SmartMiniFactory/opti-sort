#!/usr/bin/env python

# Filename:                     pub_odom_tf.py
# Creation Date:                24/05/2022
# Last Revision Date:           24/05/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	Free Univerisity of Bozen/Bolzano
# Status:                       Not working

#.............................................About wildlife_odom.py.....................................................
# This code is aimed to read data from motors' encoders and compute the tf transformations, as well as, the odometry.
# Then  it publishes them over '/mattro/tf' and '/mattro/odom' accordingly.
# some information about robot's status over the topic "/mattro/bock_status".
# At the same time, it reads the datat pusblished on the same topic "/mattro/bock_status", and converts them into a CAN
# message to control the motors' drivers.

# Inputs [subscribers]: bock_status
#                       
# Outputs [publishers]: /mattro/tf
#                       /mattro/odom'
#...........................................Included Libraries and Message Types.........................................
import rospy
import roslib
from msg import BockStatus

from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
#...........................................End of Included Libraries and Message Types..................................

#.........................................................Global Variables...............................................
speed_left_target = 0.0
speed_right_target = 0.0
gear_target = 0

speed_left = 0.0
speed_right = 0.0
    
state_of_activation = 0
random_number = 0.0
shift_value = 0
activation_code = 0
gear = 0
state_of_charge = 0.0

state = 0
#.....................................................End of Global Variables............................................

#......................................................Callback Functions ...............................................   
def BockStatusCallback(status_msg):
    global speed_left_target, speed_right_target, gear_target, speed_left, speed_right, state_of_activation, speed_right
    global random_number, shift_value, activation_code, gear, state_of_charge, state

    speed_left_target = status_msg.speed_left_target
    speed_right_target = status_msg.speed_right_target
    gear_target = status_msg.gear_target

    speed_left = status_msg.speed_left
    speed_right = status_msg.speed_right
        
    state_of_activation = status_msg.state_of_activation
    random_number = status_msg.random_number
    shift_value = status_msg.shift_value
    activation_code = status_msg.activation_code
    gear = status_msg.gear
    state_of_charge = status_msg.state_of_charge

    state = status_msg.state
#...................................................End of Callback Functions ...........................................
 
#...................................................User-defined Functions ..............................................
def update_odom(t_next, then, wheel_distance, base_frame_id, odom_frame_id):
    global speed_left, speed_right
    
    odomBroadcaster = TransformBroadcaster()
    odom_msg = Odometry()
    quaternion = Quaternion()

    # Starting position in xy plane 
    x = 0                  
    y = 0
    z = 0
    theta = 0  

    # Starting speeds in x/rotation
    dx = 0                 
    dr = 0

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if now > t_next:
            elapsed = now - then
            then = now
            elapsed = elapsed.to_sec()
            
            # Calculate the speed
            v_rx = ( speed_right + speed_left ) /2
            v_ry = 0 # we have a non-holonomic constraint (for a holonomic robot, this is non-zero)
            omega_r = ( speed_right - speed_left ) / wheel_distance # d denotes the distance between both wheels (track)

            # Calculate the travelled distance
            theta = theta + omega_r * elapsed
            x = x + (v_rx * elapsed * cos(theta))
            y = y + (v_rx * elapsed * sin(theta))
            
            # Compute the quaternion
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( theta / 2 )
            quaternion.w = cos( theta / 2 )

            # Publish the tf transformation
            odomBroadcaster.sendTransform((x, y, z), (quaternion.x, quaternion.y, quaternion.z, quaternion.w), rospy.Time.now(), base_frame_id, odom_frame_id)
            

            odom_msg.header.stamp = now
            odom_msg.header.frame_id = odom_frame_id
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = z
            odom_msg.pose.pose.orientation = quaternion
            odom_msg.child_frame_id = base_frame_id
            odom_msg.twist.twist.linear.x = dx
            odom_msg.twist.twist.linear.y = 0
            odom_msg.twist.twist.angular.z = dr

            pub1.publish(odom_msg)
    
    print("Node terminated")

    
#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    nodeRate = 10

    try:
        rospy.loginfo("Try running node")
        rospy.init_node('pub_odom_tf', anonymous=True)

        loop_rate = rospy.Rate(nodeRate)      
        base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        wheel_distance = rospy.get_param('~base_width',0.644) 

        # Define ROS publishers and Subscribers
        pub1 = rospy.Publisher("/mattro/odom", Odometry, queue_size=10)
        sub1 = rospy.Subscriber("/mattro/bock_status", BockStatus, BockStatusCallback)

        t_next = rospy.Time.now() + rospy.Duration(1.0/loop_rate)
        then = rospy.Time.now()

        update_odom(t_next, then, wheel_distance, base_frame_id, odom_frame_id)

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
#................................................End of Main Function......................................................... 
