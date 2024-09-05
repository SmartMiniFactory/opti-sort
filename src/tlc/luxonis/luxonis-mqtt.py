#!/usr/bin/env python3

####################################################### LIBRARIES #######################################################
import cv2
import keyboard
import paho.mqtt.client as mqtt
import base64
import time
import numpy as np
import depthai as dai

####################################################### VARIABLES #######################################################

# MQTT
BOKER_ID = 2
MQTT_BROKER = ["192.168.0.102", "localhost", "10.10.238.20"] # IP address of the MQTTT broker
MQTT_PORT = 1883 # Port of the MQTT Broker
MQTT_TOPIC = "optisort/luxonis/stream" # Topic on which frame will be published
MQTT_USER = "dgalli" # Username for the MQTT Broker
MQTT_PASSWORD = "dgalli" # Password for the MQTT Broker

# OpenCv
encode_param_jpg = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
encode_param_png = [cv2.IMWRITE_PNG_COMPRESSION, 0]

########################################### USER-DEFINED FUNCTIONS ######################################################

################################################## MAIN PROGRAM #########################################################

print("Connecting to MQTT broker")
client = mqtt.Client() # Create the MQTT Client
if BOKER_ID == 0: # If the borker is autorized, set username and password
    client.username_pw_set(MQTT_USER, MQTT_PASSWORD) # Setting username and password
client.connect(MQTT_BROKER[BOKER_ID], MQTT_PORT) # Establishing Connection with the Broker
print("Connected to MQTT broker")

print("Opening the camera")
pipeline = dai.Pipeline()# Create pipeline
camRgb = pipeline.create(dai.node.ColorCamera) # Define source
xoutRgb = pipeline.create(dai.node.XLinkOut) # Define output
xoutRgb.setStreamName("rgb")
# Properties
camRgb.setPreviewSize(640, 480)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
#Linking
camRgb.preview.link(xoutRgb.input)
# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    print('Connected cameras:', device.getConnectedCameraFeatures())
    # Print out usb speed
    print('Usb speed:', device.getUsbSpeed().name)
    # Bootloader version
    if device.getBootloaderVersion() is not None:
        print('Bootloader version:', device.getBootloaderVersion())
    # Device name
    print('Device name:', device.getDeviceName(), ' Product name:', device.getProductName())

    print("Starting streaming")
    print("Press 'Q' to quit")

    # Output queue will be used to get the rgb frames from the output defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while True:
        inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
        start = time.time() # Start time
        frame = inRgb.getCvFrame() # Read frame

        # Encode image to PNG format
        _frame = cv2.imencode('.png', frame, encode_param_png)[1].tobytes()
        client.publish(MQTT_TOPIC, _frame) # Publish the Frame on the Topic home/server
        cv2.imshow("Stream input", frame) # Show the frame

        end = time.time() # End time
        t = end - start
        fps = 1/t
        print("FPS: ", np.round(fps, 0), end="\r") # Print the FPS
        
        # Press q if you want to end the loop
        if cv2.waitKey(1) & keyboard.is_pressed('q'):
            print("\nqQuitting...")
            cv2.destroyAllWindows()
            break

    client.disconnect()
    print("\nStopped streaming")