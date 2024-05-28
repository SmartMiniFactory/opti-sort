#!/usr/bin/env python3

# Filename:                     can_write.py
# Creation Date:                20/04/2022
# Last Revision Date:           28/05/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Ready for testing
# Notes:

#.............................................About can_write.py.....................................................
# This code reads the datat pusblished on the topic "/mattro/bock_status", and converts them into a CAN message
# to control the motors' drivers.

# Inputs [subscribers]: /'mattro/bock_status
#                       target speeds for the two motors
# Outputs [publishers]: messages over CAN
#...........................................Included Libraries and Message Types.........................................
import rospy
import os
import can
#from can import Message
from bock_controller.msg import BockStatus
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

reading = False
#.....................................................End of Global Variables............................................

#......................................................Callback Functions ...............................................   
def BockStatusCallback(status_msg):
    global speed_left_target, speed_right_target, gear_target, speed_left, speed_right, state_of_activation, speed_right
    global random_number, shift_value, activation_code, gear, state_of_charge, state, reading

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

    reading = status_msg.running_read
    
#...................................................End of Callback Functions ...........................................
 
#...................................................User-defined Functions ..............................................
def write_can():
    global speed_left_target, speed_right_target, gear_target, speed_left, speed_right, state_of_activation, speed_right
    global random_number, shift_value, activation_code, gear, state_of_charge, state, reading

    bock_can = can.interface.Bus(channel = 'can0', bustype = 'socketcan')
    
    print("Connecting to the Bock...")
    # Send request of activation
    connect_msg = can.Message(is_extended_id=False, arbitration_id=0x195, data=[0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    bock_can.send(connect_msg)

    # Wait for "state_of_activation==2"
    while state_of_activation != 2:
        bock_can.send(connect_msg)
        rospy.sleep(0.1)
        pass
    
    # Send the activation message
    activ_msg = can.Message(is_extended_id=False, arbitration_id=0x195, data=[0x02, 0x00, 0x00, activation_code, 0x00, 0x00, 0x00, 0x00])
    bock_can.send(activ_msg)
    print("Activation message sent")
                
    # Wait for "state_of_activation==3"
    while state_of_activation != 3:
        pass

    print("Bock connected")
    while not rospy.is_shutdown():
        bock_can.send(activ_msg)
        '''
        drivemode = 0b00000011
    
        if speed_left_target > 0:
            drivemode = drivemode | 0b00010000
        else:
            drivemode = drivemode | 0b00100000
    
        if speed_right_target > 0:
            drivemode = drivemode | 0b01000000
        else:
            drivemode = drivemode | 0b10000000
            
        speed_left_H = (int(abs(speed_left_target)*10)>>8)
        speed_left_L = (int(abs(speed_left_target)*10)) & 0xff
        speed_right_H = (int(abs(speed_right_target)*10)>>8)
        speed_right_L = (int(abs(speed_right_target)*10)) & 0xff
    
        # Create a CAN message to be published
        can_msg = can.Message(is_extended_id=False, arbitration_id=0x295, data=[drivemode, speed_left_L, speed_left_H, speed_right_L, speed_right_H, gear_target, 0x00, 0x00])
        # Send the message over CAN
        bock_can.send(can_msg)
        '''

        
    # Send request of deactivation
    deact_msg = can.Message(is_extended_id=False, arbitration_id=0x195, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    bock_can.send(deact_msg)
    print("Node terminated")       
#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    nodeRate = 10

    try:
        print("Try running node")
        rospy.init_node('can_write', anonymous=True)
        loop_rate = rospy.Rate(nodeRate)

        # Define ROS publishers and Subscribers
        sub1 = rospy.Subscriber("/mattro/bock_status", BockStatus, BockStatusCallback)
        
        write_can()

    except rospy.ROSInterruptException:
        print("Node terminated")
#................................................End of Main Function......................................................... 
