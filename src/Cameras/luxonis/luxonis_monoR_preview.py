#!/usr/bin/env python3
import time

import cv2
import depthai as dai
import keyboard
import numpy as np

def clamp(num, v0, v1):
    return max(v0, min(num, v1))

sendCamConfig = False

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoRight = pipeline.create(dai.node.MonoCamera)
manipRight = pipeline.create(dai.node.ImageManip)

controlIn = pipeline.create(dai.node.XLinkIn)
configIn = pipeline.create(dai.node.XLinkIn)
manipOutRight = pipeline.create(dai.node.XLinkOut)

controlIn.setStreamName('control')
configIn.setStreamName('config')
manipOutRight.setStreamName("right")

# Crop range
topLeft = dai.Point2f(0.2, 0.2)
bottomRight = dai.Point2f(0.7, 0.7)

# Properties
monoRight.setCamera("right")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setFps(50) # Set framerate to 50 FPS
manipRight.initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y)
manipRight.setMaxOutputFrameSize(monoRight.getResolutionHeight()*monoRight.getResolutionWidth()*3)

# Linking
monoRight.out.link(manipRight.inputImage)
controlIn.out.link(monoRight.inputControl)
configIn.out.link(manipRight.inputConfig)
manipRight.out.link(manipOutRight.input)


print("Connected to Luxonis camera")

cnt = 0

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the grayscale frames
    qRight = device.getOutputQueue(manipOutRight.getStreamName(), maxSize=4, blocking=False)
    configQueue = device.getInputQueue(configIn.getStreamName())
    controlQueue = device.getInputQueue(controlIn.getStreamName())

    expTime = 500  # [microseconds] min 1, max 33000
    sensIso = 300  # min 100, max 1600
    ctrl = dai.CameraControl()
    ctrl.setAutoExposureEnable()
    ctrl.setAutoExposureLimit(500)
    controlQueue.send(ctrl)

    print("Start streaming")
    print("Press 'Q' to quit")
    print("Press 'S' to save the frame")


    while True:
        inRight = qRight.get()
        frame =inRight.getCvFrame()
        cv2.imshow("Mono R", frame)

        # Update screen (1ms pooling rate)
        if cv2.waitKey(1) and keyboard.is_pressed('q'):
            print("\nQuitting...")
            break


        image_name = "image_"+str(cnt)+".bmp"
        cv2.imwrite(image_name, frame)
        print(image_name+" saved")
        cnt += 1
        time.sleep(5)