#!/bin/python3



# Class to use the Mattro Bock platform
# Giovanni Carabin

# First install the CANbus library
# pip3 install python-can 

# In order to start-up the USB-CAN device type the following commands (linux):
#os.system('sudo ip link set can0 type can bitrate 500000')
#os.system('sudo ip link set up can0 ')

# Speed is set as percentage of the maximum speed (i.e. 100% = 4500rpm). Note that the gear can assume values from 0 (park mode) to 4. Anyway, in this mode gears from 1 to 4 have no differences.



import os
import can
from can import Message
import time
import threading



class MattroBock:
    
    def __init__(self, channel = 'can0'):
        self.bock_can = can.interface.Bus(channel = channel, bustype = 'socketcan') #'socketcan_ctype'
        
        self.speed_left_target = 0
        self.speed_right_target = 0
        self.gear_target = 0
        
        self.state_of_activation = 0
        self.random_number = 0
        self.shift_value = 0
        self.activation_code = 0
        self.gear = 0
        self.state_of_charge = 0
        self.speed_left = 0
        self.speed_right = 0
        
        self._running_read = 1
        self._running_write = 1
        
        # Set read and write threading
        self.read_thr = threading.Thread(name='read_threading', target=self.read_threading)
        self.write_thr = threading.Thread(name='write_threading', target=self.write_threading)
        self.read_thr.start()
    
    
    def connect(self):
        # Send request of activation
        msg = Message(is_extended_id=False, arbitration_id=0x195, data=[0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])#, extended_id=False)
        self.bock_can.send(msg)
        
        # Wait for "state_of_activation==2"
        while self.state_of_activation != 2:
            self.bock_can.send(msg)
            time.sleep(0.1)
            pass
        
        # Send the activation message
        msg = Message(is_extended_id=False, arbitration_id=0x195, data=[0x02, 0x00, 0x00, self.activation_code, 0x00, 0x00, 0x00, 0x00])#, extended_id=False)
        self.bock_can.send(msg)
        
        # Wait for "state_of_activation==3"
        while self.state_of_activation != 3:
            pass
        
        # Start sending the control signal
        self.write_thr.start()
    
    
    def disconnect(self):
        # Set the speed to zero
        self.speed_left_target = 0
        self.speed_right_target = 0
        
        # Put the robot in park mode as soon the speed is zero
        while(self.speed_left>0 or self.speed_right>0):
            pass
        
        self.gear_target = 0
        while(self.gear!=0):
            pass
        
        # Stop all the threads
        self._running_read = 0
        time.sleep(1.0)
        self._running_write = 0
        
    
    
    
    def read_threading(self):
        while self._running_read:
            # Read a message from can bus
            msg = self.bock_can.recv()
            
            # Parse messages
            if msg.arbitration_id == 0x215:
                self.state_of_activation = msg.data[0]
                self.activation_code = (msg.data[1]>>msg.data[2])
                self.gear = msg.data[5]
                self.state_of_charge = ((msg.data[7]<<8)+msg.data[6])/10.
            elif msg.arbitration_id == 0x315:
                self.speed_left = (msg.data[3]<<24)+(msg.data[2]<<16)+(msg.data[1]<<8)+msg.data[0]
                self.speed_right = (msg.data[7]<<24)+(msg.data[6]<<16)+(msg.data[5]<<8)+msg.data[4]
    
    
    def write_threading(self):
        while self._running_write:
            drivemode = 0b00000011
            if(self.speed_left_target>0):
                drivemode = drivemode | 0b00010000
            else:
                drivemode = drivemode | 0b00100000
            if(self.speed_right_target>0):
                drivemode = drivemode | 0b01000000
            else:
                drivemode = drivemode | 0b10000000
            
            speed_left_H = (int(abs(self.speed_left_target)*10)>>8)
            speed_left_L = (int(abs(self.speed_left_target)*10)) & 0xff
            speed_right_H = (int(abs(self.speed_right_target)*10)>>8)
            speed_right_L = (int(abs(self.speed_right_target)*10)) & 0xff
            
            msg = Message(is_extended_id=False, arbitration_id=0x295, data=[drivemode, speed_left_L, speed_left_H, speed_right_L, speed_right_H, self.gear_target, 0x00, 0x00])#, extended_id=False)
            self.bock_can.send(msg)
            
            time.sleep(0.005)
        
        # Send request of deactivation
        msg = Message(is_extended_id=False, arbitration_id=0x195, data=[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])#, extended_id=False)
        self.bock_can.send(msg)


