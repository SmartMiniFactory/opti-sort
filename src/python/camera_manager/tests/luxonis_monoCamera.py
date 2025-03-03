#!/usr/bin/env python3

import depthai as dai
import cv2

pipeline = dai.Pipeline()

'''
MonoCamera node is a source of ImgFrame. You can control in at runtime with the inputControl. 
Some DepthAI modules don't have mono camera(s). 
Two mono cameras are used to calculate stereo depth (with StereoDepth node).

The 3A - Auto-Exposure (AE), Auto-White Balance (AWB), and Auto-Focus (AF) - algorithms are used to optimize image quality and run directly on RVC. 
By default, these settings are in AUTO mode, with limits (e.g., min/max exposure) specific to each sensor (see supported sensors for details).
You can manually control these settings either by following the steps in RGB camera control example or by using the cam_test.py script.
Stereo Cameras: Sensors share the same I2C bus, ensuring synchronized 3A settings automatically (AWB, AE).
Independent Sensors: On setups like OAK FFC or OAK-D-LR, where each sensor has its own I2C, the 3a-follow feature can be used to synchronize 3A settings from one sensor to others.
'''


# Define sources and outputs
monoR = pipeline.create(dai.node.MonoCamera)
monoR.setCamera("right")
monoR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)

monoL = pipeline.create(dai.node.MonoCamera)
monoL.setCamera("left")
monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)


# Linking ---------------------------------
monoOutLeft = pipeline.create(dai.node.XLinkOut)
monoOutLeft.setStreamName("monoLeft")
monoOutRight = pipeline.create(dai.node.XLinkOut)

monoOutRight.setStreamName("monoRight")
monoR.out.link(monoOutRight.input)
monoL.out.link(monoOutLeft.input)

with dai.Device(pipeline) as device:

    qLeft = device.getOutputQueue(name="monoLeft", maxSize=4, blocking=False)
    qRight = device.getOutputQueue(name="monoRight", maxSize=4, blocking=False)

    while True:
        if qLeft.has():
            cv2.imshow("LLLLLLEFTT", qLeft.get().getCvFrame())
        if qRight.has():
            cv2.imshow("RRRRIGHT", qRight.get().getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break
