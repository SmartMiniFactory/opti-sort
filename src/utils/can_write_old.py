#!/usr/bin/env python3

# Filename:                     bock_write.py
# Creation Date:                30/04/2022
# Last Revision Date:           01/06/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Up and Running
# Notes:

#.............................................About can_write.py.....................................................
# This is a rough implementation of the python scripts written by Carabin, simply converted as ROS node
# to control the motors' drivers.

# Inputs [subscribers]: /'mattro/bock_status
#                       
# Outputs [publishers]: messages over CAN
#...........................................Included Libraries and Message Types.........................................
import rospy

from bock_controller.msg import BockStatus
from utils import mattro_bock
#...........................................End of Included Libraries and Message Types..................................

#.........................................................Global Variables...............................................
speed_left_target = 0.0
speed_right_target = 0.0
gear_target = 0

state = 0
#.....................................................End of Global Variables............................................

#......................................................Callback Functions ...............................................   
def BockStatusCallback(status_msg):
    global speed_left_target, speed_right_target, gear_target, state

    speed_left_target = status_msg.speed_left_target
    speed_right_target = status_msg.speed_right_target
    gear_target = status_msg.gear_target

    state = status_msg.state
#...................................................End of Callback Functions ...........................................
 
#...................................................User-defined Functions ..............................................
def bock_control():
    global speed_left_target, speed_right_target, gear_target, state

    loop_rate = rospy.Rate(10)
    bock = mattro_bock.MattroBock()
    # Connect to the robot
    print("Connecting to the Bock...")
    bock.connect()
    print("Bock connected")
    print("CanWriteNode: up and running")

    while not rospy.is_shutdown():
        # Set the speed
        bock.gear_target = 1
        bock.speed_left_target = speed_left_target
        bock.speed_right_target = speed_right_target
        loop_rate.sleep()
    
    # Stop the bock
    #bock.speed_left_target = 0
    #bock.speed_right_target = 0


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
        # Define ROS publishers and Subscribers
        sub1 = rospy.Subscriber("/mattro/bock_status", BockStatus, BockStatusCallback)
        
        bock_control()

    except rospy.ROSInterruptException:
        print("Node terminated")
#................................................End of Main Function......................................................... 