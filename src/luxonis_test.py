#!/usr/bin/env python3
####################################################### LIBRARIES #######################################################
import cv2
import depthai as dai
import numpy as np
import time
import keyboard
import paho.mqtt.client as mqtt
import base64
import json
import datetime

####################################################### VARIABLES #######################################################

# MQTT
MQTT_BROKER = "localhost" # IP address of the MQTTT broker
MQTT_PORT = 1883 # Port of the MQTT Broker
MQTT_TOPIC = "optisort/luxonis/stream" # Topic on which frame will be published

# OpenCV
encode_param_png = [cv2.IMWRITE_PNG_COMPRESSION, 0]
encode_param_jpg = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

########################################### USER-DEFINED FUNCTIONS ######################################################

# Encode image to Json format
def im2json(imdata):
    jstr = json.dumps({"image": base64.b64encode(imdata).decode('ascii'), "timestamp": datetime.datetime.now().isoformat()})
    return jstr

################################################## MAIN PROGRAM #########################################################

print("Connecting to MQTT broker")
client = mqtt.Client() # Create the MQTT Client
client.connect(MQTT_BROKER, MQTT_PORT) # Establishing Connection with the Broker
print("Connected to MQTT broker")

print("Connecting to Luxonis OAK-D camera")
# Create pipeline for the Luxonis OAK-D camera
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
xoutLeft = pipeline.create(dai.node.XLinkOut)
xoutLeft.setStreamName('left')

# Properties
monoLeft.setCamera("left")
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

# Linking
monoLeft.out.link(xoutLeft.input)
print("Creating the pipeline...")

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the grayscale frames from the outputs defined above
    qLeft = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    print("Connected to Luxonis OAK-D camera")
    print("Start streaming")
    print("Press 'Q' to quit")

    while True:
        start = time.time() # Start time
        
        # Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        inLeft = qLeft.tryGet()

        if inLeft is not None:
            frame_left = inLeft.getCvFrame()
            
            # Resize the image by a half
            frame = cv2.resize(frame_left,(0,0),fx=0.5, fy=0.5)
        
            # Encode image to PNG format
            _frame = cv2.imencode('.png', frame, encode_param_png)[1].tobytes()
    
            # Publish the Frame on the Topic
            client.publish(MQTT_TOPIC, im2json(_frame))
     
            # Show the frame
            cv2.imshow("Luxonis Left Camera Stream", frame)
            
            end = time.time() # End time
            t = end - start
            if t != 0:
                fps = 1/t
            else :
                fps = 0
            
            print("FPS Luxonis: ", np.round(fps, 0), end="\r") # Print the FPS

    
        # Press q if you want to end the loop
        if cv2.waitKey(1) & keyboard.is_pressed('q'):
            print("\nqQuitting...")
            break

cv2.destroyAllWindows()

client.disconnect()
print("\nStopped streaming")

