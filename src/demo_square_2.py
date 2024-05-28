#!/usr/bin/env python3

# Filename:                     test_rotate.py
# Creation Date:                12/08/2022
# Last Revision Date:           12/08/2022
# Author(s) [email]:			Davide Galli [dgalli@unibz.it]
# Revisor(s) {Date}:        	
# Organization/Institution:	    Free Univerisity of Bozen/Bolzano
# Status:                       Up and Running
# Notes:                        

#.............................................About can_read.py.....................................................
# This code is aimed to make the mattro rovo follow a squared path width an edge lenght of 2 [m]. Position is computed
# using odoemtry, while the orientation is taken from the IMU.
#
#
# Inputs [subscribers]: /filter/quaternion
#                       /mattro/odom
#                       
# Outputs [publishers]: /mattro/cmd_vel
#                  
#...........................................Included Libraries and Message Types.........................................
import rospy
import numpy as np
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from simple_pid import PID
#...........................................End of Included Libraries and Message Types..................................

#.........................................................Global Variables...............................................
quat = np.zeros(4)
robot_position = np.zeros(2)
#.....................................................End of Global Variables............................................

#.........................................................Global Constants...............................................
ERROR_THRESHOLD = 0.2 # [deg]
LENGTH_EDGE = 2.0 # [m]
#.....................................................End of Global Constants............................................

#......................................................Callback Functions ...............................................   
def ImuCallback(msg): # Read data from the IMU
    global quat

    quat[0] = msg.quaternion.x
    quat[1] = msg.quaternion.y
    quat[2] = msg.quaternion.z
    quat[3] = msg.quaternion.w

def OdometryCallback(msg):
    global robot_position

    robot_position[0] = msg.pose.pose.position.x
    robot_position[1] = msg.pose.pose.position.y
#...................................................End of Callback Functions ...........................................

#...................................................User-defined Functions ..............................................
def Quaternion2Euler(quat):

    x = quat[0]
    y = quat[1]
    z = quat[2]
    w = quat[3]

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    euler = np.array([X, Y, Z])
    
    return euler

def mattro_square(loop_rate):
    global header_info, quat, robot_position
    print("DemoSquareNode: up and running")

    # Create a Twist message
    cmd_vel_msg = Twist()

    pid = PID(0.6, 0.08, 0.2) # 0.5 0.5 0.1

    state = 0
    distance = 0.0

    while not rospy.is_shutdown():

        euler = Quaternion2Euler(quat)
        
        if state == 0:
            x_start = robot_position[0]
            y_start = robot_position[1]
            start = np.array((x_start, y_start))
            #print("Starting X: ", x_start, " Starting Y: ", y_start)
            rospy.sleep(0.5)
            state = 1
        
        if state == 1:
            x_current = robot_position[0]
            y_current = robot_position[1]
            current = np.array((x_current, y_current))

            #print("Current position: ", current)
            distance = np.linalg.norm(current - start)
            print("Distance:", distance, end = "\r")

            if distance <= LENGTH_EDGE:
                cmd_vel_msg.linear.x = 0.2
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.linear.z = 0.0  

                cmd_vel_msg.angular.x = 0.0
                cmd_vel_msg.angular.y = 0.0
                cmd_vel_msg.angular.z = 0.0

            else:     
                state = 2

        if state == 2:
            target_angle = euler[2] - 90 # [deg]
            if target_angle >= 180:
                target_angle -= 360
            elif target_angle <= -180:
                target_angle += 360

            print("Target Yaw: ", target_angle)
            pid.setpoint = target_angle
            pid.output_limits = (- 0.3, + 0.3)    # Output value will be between -0.3 and +0.3 [rad/s]

            state = 3
        
        if state == 3:

            current_angle = euler[2]
            error = np.abs(target_angle - current_angle)
            
            if error <= ERROR_THRESHOLD:
                x_start = robot_position[0]
                y_start = robot_position[1]
                start = np.array((x_start, y_start))
                state = 4
            print("Current Yaw: ", current_angle, "Current Error: ", error, end = "\r")

            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = pid(current_angle)
        
        if state == 4:
            x_current = robot_position[0]
            y_current = robot_position[1]
            current = np.array((x_current, y_current))

            #print("Current position: ", current)
            distance = np.linalg.norm(current - start)
            print("Distance:", distance, end = "\r")
            if  distance <= LENGTH_EDGE:
                cmd_vel_msg.linear.x = 0.2
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.linear.z = 0.0

                cmd_vel_msg.angular.x = 0.0
                cmd_vel_msg.angular.y = 0.0
                cmd_vel_msg.angular.z = 0.0
                #print("Moving forward ...", end = "\r")
            else:
                state = 5

        if state == 5:
            target_angle = target_angle - 90 # [deg]
            if target_angle >= 180:
                target_angle -= 360
            elif target_angle <= -180:
                target_angle += 360
                
            print("Target Yaw: ", target_angle)
            pid.setpoint = target_angle
            pid.output_limits = (- 0.3, + 0.3)    # Output value will be between -0.3 and +0.3 [rad/s]

            state = 6
        
        if state == 6:

            current_angle = euler[2]
            error = np.abs(target_angle - current_angle)
            
            if error <= ERROR_THRESHOLD:
                x_start = robot_position[0]
                y_start = robot_position[1]
                start = np.array((x_start, y_start))
                
                state = 7
            
            print("Current Yaw: ", current_angle, "Current Error: ", error, end = "\r")

            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = pid(current_angle)
        
        if state == 7:
            x_current = robot_position[0]
            y_current = robot_position[1]
            current = np.array((x_current, y_current))

            #print("Current position: ", current)
            distance = np.linalg.norm(current - start)
            print("Distance:", distance, end = "\r")
            if  distance <= LENGTH_EDGE:
                cmd_vel_msg.linear.x = 0.2
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.linear.z = 0.0

                cmd_vel_msg.angular.x = 0.0
                cmd_vel_msg.angular.y = 0.0
                cmd_vel_msg.angular.z = 0.0
                #print("Moving forward ...", end = "\r")
            else:
                state = 8

        if state == 8:
            target_angle = target_angle - 90 # [deg]
            if target_angle >= 180:
                target_angle -= 360
            elif target_angle <= -180:
                target_angle += 360
                
            print("Target Yaw: ", target_angle)
            pid.setpoint = target_angle
            pid.output_limits = (- 0.3, + 0.3)    # Output value will be between -0.3 and +0.3 [rad/s]

            state = 9
        
        if state == 9:

            current_angle = euler[2]
            error = np.abs(target_angle - current_angle)
            
            if error <= ERROR_THRESHOLD:
                x_start = robot_position[0]
                y_start = robot_position[1]
                start = np.array((x_start, y_start))
                state = 10
            print("Current Yaw: ", current_angle, "Current Error: ", error, end = "\r")

            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = pid(current_angle)
        
        if state == 10:
            x_current = robot_position[0]
            y_current = robot_position[1]
            current = np.array((x_current, y_current))

            #print("Current position: ", current)
            distance = np.linalg.norm(current - start)
            print("Distance:", distance, end = "\r")
            if  distance <= LENGTH_EDGE:
                cmd_vel_msg.linear.x = 0.2
                cmd_vel_msg.linear.y = 0.0
                cmd_vel_msg.linear.z = 0.0

                cmd_vel_msg.angular.x = 0.0
                cmd_vel_msg.angular.y = 0.0
                cmd_vel_msg.angular.z = 0.0
                #print("Moving forward ...", end = "\r")

            else:
                state = 11

        if state == 11:
            target_angle = target_angle - 90 # [deg]
            if target_angle >= 180:
                target_angle -= 360
            elif target_angle <= -180:
                target_angle += 360
                
            print("Target Yaw: ", target_angle)
            pid.setpoint = target_angle
            pid.output_limits = (- 0.3, + 0.3)    # Output value will be between -0.3 and +0.3 [rad/s]

            state = 12
        
        if state == 12:

            current_angle = euler[2]
            error = np.abs(target_angle - current_angle)
            
            if error <= ERROR_THRESHOLD:
                state = 13
            print("Current Yaw: ", current_angle, "Current Error: ", error, end = "\r")

            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = pid(current_angle)


        if state == 13:
            print("Target succesfully reached", end = "\r")
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            cmd_vel_msg.angular.x = 0.0
            cmd_vel_msg.angular.y = 0.0
            cmd_vel_msg.angular.z = 0.0

            x_start = robot_position[0]
            y_start = robot_position[1]
            start = np.array((x_start, y_start))

            x_current = robot_position[0]
            y_current = robot_position[1]
            current = np.array((x_current, y_current))

            distance = 0.0
        
        # Publish the ROS message
        pub1.publish(cmd_vel_msg)
        loop_rate.sleep()

    # Publish the ROS message
    print("Shutting down ...")
    print("\n Node terminated")
#.............................................End of User-defined Functions ..............................................

#......................................................Main Function......................................................
if __name__ == '__main__':
    nodeRate = 10

    try:
        print("Try running node")
        rospy.init_node('demo_square', anonymous=True)
        loop_rate = rospy.Rate(nodeRate)

        # Define ROS publishers and Subscribers
        sub1 = rospy.Subscriber("/filter/quaternion", QuaternionStamped, ImuCallback)
        sub2 = rospy.Subscriber("/mattro/odom", Odometry, OdometryCallback)
        pub1 = rospy.Publisher("/mattro/cmd_vel", Twist, queue_size = 10)
        
        mattro_square(loop_rate)

    except rospy.ROSInterruptException:
        print("Node terminated")
#................................................End of Main Function......................................................... 
