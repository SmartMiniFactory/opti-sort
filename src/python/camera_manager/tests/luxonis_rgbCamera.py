#!/usr/bin/env python3

import depthai as dai
import cv2

pipeline = dai.Pipeline()

'''
Camera node is a source of ImgFrame. You can control in at runtime with the InputControl and InputConfig
It aims to unify the Color Camera and MonoCamera into one node.

Compared to Color Camera node, Camera node:
- Supports cam.setSize(), which replaces both cam.setResolution() and cam.setIspScale(). 
  Camera node will automatically find resolution that fits best, and apply correct scaling to achieve user-selected size
- Supports cam.setCalibrationAlpha(), example here: Undistort camera stream
- Supports cam.loadMeshData() and cam.setMeshStep(), which can be used for custom image warping (undistortion, perspective correction, etc.)
- Automatically undistorts camera stream if HFOV of the camera is greater than 85°. 
  You can disable this with: cam.setMeshSource(dai.CameraProperties.WarpMeshSource.NONE).

Besides points above, compared to MonoCamera node, Camera node:
- Doesn't have out output, as it has the same outputs as ColorCamera (raw, isp, still, preview, video). 
  This means that preview will output 3 planes of the same grayscale frame (3x overhead), 
  and isp / video / still will output luma (useful grayscale information) + chroma (all values are 128), 
  which will result in 1.5x bandwidth overhead
  
https://docs.luxonis.com/software/depthai-components/nodes/camera/
'''

# Define sources and outputs
cam: dai.node.Camera = pipeline.create(dai.node.Camera) # rgb camera; can output both ISP and VIDEO
cam.setBoardSocket(dai.CameraBoardSocket.CAM_B)
cam.setSize((1280, 800)) # setSize replaces both cam.setResolution() and cam.setIspScale() # Camera node will automatically find resolution that fits best and apply correct scaling to achieve user-selected size
#cam.setPreviewSize(300, 300)

# Create node connections
configIn = pipeline.create(dai.node.XLinkIn)
ispOut = pipeline.create(dai.node.XLinkOut) # distorted, but interacts with 3A algorithm; highest possible res
videoOut = pipeline.create(dai.node.XLinkOut) # automatically undistorted; cropped to 4k (3840x2160) from ISP
previewOut = pipeline.create(dai.node.XLinkOut) # automatically undistorted; 1:1 aspect ratio cropped to from VIDEO

manip = pipeline.create(dai.node.ImageManip) # used to crop image

'''
Preferred approach:
streaming the video output and cropping the FOV mantaining the highest possible resolution
Crop will be done at a 1:1 aspect ratio, which will be mandatory also for other cameras


'''

# Stream names
configIn.setStreamName('config')
videoOut.setStreamName("video")
ispOut.setStreamName("isp")
previewOut.setStreamName("prev")

# Linking
configIn.out.link(cam.inputConfig)
cam.video.link(videoOut.input)
cam.isp.link(ispOut.input)
cam.preview.link(previewOut.input)

# TODO: greyscale

#cam.video.link(manip.inputImage)
#configIn.out.link(manip.inputConfig)
#manip.out.link(videoOut.input)

# Set up cropping with ImageManip
manip.initialConfig.setCropRect(0.3046875, 0.1875, 0.6953125, 0.8125)  # Normalized coordinates
manip.setMaxOutputFrameSize(1280 * 800 * 3)

with dai.Device(pipeline) as device:

    configQueue = device.getInputQueue('config')

    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
    isp = device.getOutputQueue(name="isp", maxSize=1, blocking=False)
    prev = device.getOutputQueue(name="prev", maxSize=1, blocking=False)

    while True:

        cfg = dai.ImageManipConfig()
        cfg.setCropRect(0.3046875, 0.1875, 0.6953125, 0.8125)
        configQueue.send(cfg)

        if video.has():
            cv2.imshow("video", video.get().getCvFrame())
        if isp.has():
            cv2.imshow("isp", isp.get().getCvFrame())
        if prev.has():
            cv2.imshow("prev", prev.get().getCvFrame())
        if cv2.waitKey(1) == ord('q'):
            break
