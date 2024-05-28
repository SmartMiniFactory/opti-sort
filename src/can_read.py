#!/usr/bin/env python3

# Filename:                     can_read.py
# Creation Date:                20/04/2022
# Last Revision Date:           16/09/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Up and Running
# Notes:                        
#
#.............................................About can_read.py.....................................................
# This code is aimed to connect the ROS network and the CAN network. It reads the data transmitted over CAN and publishes
# some information about robot's status over the topic "/mattro/bock_status".
#
#
# Inputs [subscribers]: CAN message
#                       target speeds for the two motors
# Outputs [publishers]: bock_status
#                       messages over CAN
#...........................................Included Libraries and Message Types.........................................
import rospy
import math
import can
from bock_controller.msg import BockStatus
#...........................................End of Included Libraries and Message Types..................................
 
#...................................................User-defined Functions ..............................................
def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:          # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)                 # compute negative value
    return ((val * 0.35 * math.pi)/(16 * 60))    # return positive value as is

def read_can():
    
    bock_can = can.interface.Bus(channel = 'can0', bustype = 'socketcan') #socketcan_ctypes
    print("CanReadNode: up and running")

    # Create a BockStatus message
    status_msg = BockStatus()
    
    while not rospy.is_shutdown():
            
        # Read a message from can bus
        can_msg = bock_can.recv()

        # Parse the message
        if can_msg.arbitration_id == 0x215:
            status_msg.state_of_activation = can_msg.data[0]
            status_msg.random_number = can_msg.data[1]
            status_msg.shift_value = can_msg.data[2]
            status_msg.activation_code = (can_msg.data[1]>>can_msg.data[2])
            status_msg.gear = can_msg.data[5]
            status_msg.state_of_charge = ((can_msg.data[7]<<8) + can_msg.data[6])/10.
        
        elif can_msg.arbitration_id == 0x315:

            hex_msg = can_msg.data.hex()

            left_hex_string = hex_msg[6:8] + hex_msg[4:6] + hex_msg[2:4] + hex_msg[:2]
            right_hex_string = hex_msg[14:16] + hex_msg[12:14] + hex_msg[10:12] + hex_msg[8:10]

            status_msg.speed_left = -(twos_comp(int(left_hex_string,16), 32))
            status_msg.speed_right = twos_comp(int(right_hex_string,16), 32)

        status_msg.running_read = True
        status_msg.state = 3
        # Publish the ROS message
        pub1.publish(status_msg)

    print("\n Node terminated")
#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    nodeRate = 10

    try:
        print("Try running node")
        rospy.init_node('can_read', anonymous=True)
        loop_rate = rospy.Rate(nodeRate)

        # Define ROS publishers and Subscribers
        pub1 = rospy.Publisher("/mattro/bock_status", BockStatus, queue_size = 10)
        
        read_can()

    except rospy.ROSInterruptException:
        print("Node terminated")
#................................................End of Main Function......................................................... 