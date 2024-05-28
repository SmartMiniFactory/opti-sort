#!/bin/python3


import mattro_bock
import time
import math



def track_speed(V, omega):
    
    w = 0.936
    
    vr = V + omega*w/2
    vl = V - omega*w/2
    
    
    nr = vr*7*60/(0.34*math.pi)*100/4500
    nl = vl*7*60/(0.34*math.pi)*100/4500

    print(vr)
    print(vl)
    return (nr, nl)




# Robot object
bock = mattro_bock.MattroBock()

# Connect to the robot
print("Connecting to the Bock...")
bock.connect()
print("Bock connected.")

# Print the SOC level
print("SOC = %f" %(bock.state_of_charge))


# Set the speed
V = 0.5 # [m/s]
omega = -30 * math.pi/180 # [rad/s]
[nr, nl] = track_speed(V, omega)

bock.gear_target = 1
bock.speed_left_target = nl
bock.speed_right_target = nr

time.sleep(10.0)

# Stop the bock
bock.speed_left_target = 0
bock.speed_right_target = 0


# Disconnect from the bock
bock.disconnect()
print("Bock disconnected.")

