#! /bin/bash

# Date: 05.08.2022
# Author: Davide Galli [dgalli@unibz.it]
# Purpose: Automatically set up the ROS environment on start-up

# Set up the CAN communication with motor drivers
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# Set up the USB communication to the IMU
sudo chmod a+rw /dev/ttyUSB0

# Run ROS environment
roscore
