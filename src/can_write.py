#!/usr/bin/env python3

# Filename:                     bock_write.py
# Creation Date:                30/04/2022
# Last Revision Date:           01/06/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Up and Running
# Notes:
#
#.............................................About can_write.py.....................................................
# This is a rough implementation of the python scripts written by Carabin, simply converted as ROS node
# to control the motors' drivers.

# Inputs [subscribers]: /'mattro/bock_status
#                       
# Outputs [publishers]: messages over CAN
#...........................................Included Libraries and Message Types.........................................
import rospy

from bock_controller.msg import BockStatus
from geometry_msgs.msg import Twist
from utils import mattro_bock

import numpy as np
#...........................................End of Included Libraries and Message Types..................................

#.........................................................Global Variables...............................................
speed_left_target = 0.0
speed_right_target = 0.0
gear_target = 0

dx = 0.0
dr = 0.0

state = 0
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

def bock_control(loop_rate, wheel_space):
    global dx, dr

    bock = mattro_bock.MattroBock()
    # Connect to the robot
    print("Connecting to the Bock...")
    bock.connect()
    print("Bock connected")
    print("CanWriteNode: up and running")

    while not rospy.is_shutdown():
        # Set the speed
        bock.gear_target = 1
        bock.speed_left_target = compute_rpm_percentage(1.0 * dx - dr * wheel_space / 2)
        bock.speed_right_target = compute_rpm_percentage(1.0 * dx + dr * wheel_space / 2)
        loop_rate.sleep()

    # Disconnect from the bock
    bock.disconnect()
    print("Bock disconnected.")
    print("Node terminated")       
#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    nodeRate = 10

    try:
        print("Try running node")
        rospy.init_node('bock_bridge', anonymous=True)

        # Get ROS parameters from the launch file
        rate = rospy.get_param("~rate", 10)
        loop_rate = rospy.Rate(rate)
        wheel_space = rospy.get_param("~base_width", 0.644)

        # Define ROS publishers and Subscribers
        sub2 = rospy.Subscriber("/mattro/cmd_vel", Twist, TwistCallback)
        
        bock_control(loop_rate, wheel_space)

    except rospy.ROSInterruptException:
        print("Node terminated")
#................................................End of Main Function......................................................... 