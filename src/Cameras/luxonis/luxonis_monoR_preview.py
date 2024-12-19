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
bottomRight = dai.Point2f(0.8, 0.8)

# Properties
monoRight.setCamera("right")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
manipRight.initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y)
manipRight.setMaxOutputFrameSize(monoRight.getResolutionHeight()*monoRight.getResolutionWidth()*3)

# Linking
monoRight.out.link(manipRight.inputImage)
controlIn.out.link(monoRight.inputControl)
configIn.out.link(manipRight.inputConfig)
manipRight.out.link(manipOutRight.input)

# Defaults and limits for manual focus/exposure controls
expTime = 100 # [micro s]
expMin = 1
expMax = 33000

sensIso = 500
sensMin = 100
sensMax = 1600

isoRange = [100, 200, 300, 400, 500, 600,  700, 800, 900, 1000, 1100, 1200, 1300 , 1400, 1500, 1600]
expRange = np.linspace(expMin, expMax, num=50)
print("Connected to IDS camera")

expTime = clamp(expTime, expMin, expMax)
sensIso = clamp(sensIso, sensMin, sensMax)
print("Setting manual exposure, time:", expTime, "iso:", sensIso)
ctrl = dai.CameraControl()
ctrl.setManualExposure(expTime, sensIso)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queues will be used to get the grayscale frames
    qRight = device.getOutputQueue(manipOutRight.getStreamName(), maxSize=4, blocking=False)
    configQueue = device.getInputQueue(configIn.getStreamName())
    controlQueue = device.getInputQueue(controlIn.getStreamName())

    print("Start streaming")
    print("Press 'Q' to quit")

    #while True:
    for i in range(len(expRange)):
        inRight = qRight.get()
        cv2.imshow("Mono R", inRight.getCvFrame())

        expTime = clamp(expRange[i], expMin, expMax)
        sensIso = clamp(sensIso, sensMin, sensMax)
        print("Setting manual exposure, time:", expTime, "iso:", sensIso)
        ctrl = dai.CameraControl()
        ctrl.setManualExposure(expTime, sensIso)
        controlQueue.send(ctrl)
        time.sleep(1)  # Wait 2 seconds



        # Update screen (1ms pooling rate)
        if cv2.waitKey(1) & keyboard.is_pressed('q'):
            print("\nqQuitting...")
            break