#!/usr/bin/env python3

# Filename:                     twist_to_motors.py
# Creation Date:                23/04/2022
# Last Revision Date:           01/06/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Up and Running

#.............................................About wildlife_odom.py.....................................................
# This code convets a Twist message from '/mattro/cmd_vel' into the motors' target velocities published as BockStatus 
# messages over '/mattro/bock_status'. Inbetwee, it also runs a PID controller to set the target velocities.
#
# Inputs [subscribers]: /mattro/cmd_vel
# Outputs [publishers]: bock_status
#...........................................Included Libraries and Message Types.........................................
import rospy
from bock_controller.msg import BockStatus
from geometry_msgs.msg import Twist
import numpy as np
#...........................................End of Included Libraries and Message Types..................................

#.........................................................Global Variables...............................................
dx = 0.0
dr = 0.0
#.....................................................End of Global Variables............................................

#......................................................Callback Functions ...............................................   
def TwistCallback(msg): # Read data form /mattro/cmd_vel
    global dx, dr
    
    dx = msg.linear.x
    dr = msg.angular.z

#...................................................End of Callback Functions ...........................................
 
#...................................................User-defined Functions ..............................................
def remap_percentage(x):

    oMin = 0
    oMax = 450
    
    nMin = 0
    nMax = 100

    #range check
    if oMin == oMax:
        print("Warning: Zero input range")
        return None

    if nMin == nMax:
        print("Warning: Zero output range")
        return None

    #check reversed input range
    reverseInput = False
    oldMin = min( oMin, oMax )
    oldMax = max( oMin, oMax )
    if not oldMin == oMin:
        reverseInput = True

    #check reversed output range
    reverseOutput = False   
    newMin = min( nMin, nMax )
    newMax = max( nMin, nMax )
    if not newMin == nMin :
        reverseOutput = True

    portion = (x-oldMin)*(newMax-newMin)/(oldMax-oldMin)
    if reverseInput:
        portion = (oldMax-x)*(newMax-newMin)/(oldMax-oldMin)

    result = portion + newMin
    if reverseOutput:
        result = newMax - portion

    return result

def compute_rpm_percentage(velocity):

    # Set local varibles
    drive_wheel_diameter = 0.35 # Heavy Duty => 0.35 [m]
    gear_ratio = 16 # Heavy Duty => i16

    rpm = (velocity * gear_ratio * 60)/(drive_wheel_diameter * 3.14 * 3.6)
    return remap_percentage(rpm)

def motor_spin(loop_rate, wheel_space):
    global dx, dr
    
    print("TwistToMotorNode: up and running")

    # Create a BockStatus message
    status_msg = BockStatus()
    
    while not rospy.is_shutdown():# and state == 3:

        # Compute the righ and left track speeds

        status_msg.speed_right_target = compute_rpm_percentage(1.0 * dx + dr * wheel_space / 2)
        status_msg.speed_left_target = compute_rpm_percentage(1.0 * dx - dr * wheel_space / 2)
        status_msg.gear_target = 1
        #print("Right speed: ", status_msg.speed_right_target, "Left speed: ", status_msg.speed_left_target, end = "\r")
        # Public them over /mattro/bock_status
        pub1.publish(status_msg)

        # Wait till the end of the loop
        loop_rate.sleep()

    rospy.loginfo("Node terminated")

#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    try:
        rospy.loginfo("Try running node")
        rospy.init_node('twist_to_motor_speed', anonymous = True)

        # Get ROS parameters from the launch file
        rate = rospy.get_param("~rate", 10)
        loop_rate = rospy.Rate(rate)
        wheel_space = rospy.get_param("~base_width", 0.644)


        # Define ROS publishers and Subscribers
        pub1 = rospy.Publisher("/mattro/bock_status", BockStatus, queue_size = 10)
        #sub1 = rospy.Subscriber("/mattro/bock_status", BockStatus, BockStatusCallback)
        sub2 = rospy.Subscriber("/mattro/cmd_vel", Twist, TwistCallback)

        motor_spin(loop_rate, wheel_space)

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
#................................................End of Main Function......................................................... 