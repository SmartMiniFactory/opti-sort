#!/usr/bin/env python3

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

import depthai as dai
import cv2

pipeline = dai.Pipeline()

# Define sources and outputs
cam = pipeline.create(dai.node.Camera)  # RGB camera
cam.setBoardSocket(dai.CameraBoardSocket.CAM_B)
cam.setSize((1280, 800))  # Initial resolution selection

# ImageManip node for cropping an AOI to 1:1 aspect ratio
manip = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setCropRect(0.3,0.1,0.7,0.9)
#manip.initialConfig.setCropRect(0.2, 0.0, 0.8, 1.0)  # Crop a central AOI with a 1:1 aspect ratio
manip.initialConfig.setResize(800, 800)  # Resize after cropping
manip.setMaxOutputFrameSize(800 * 800 * 3)  # Assuming max square crop is 800x800
manip.setKeepAspectRatio(False)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)  # Convert to greyscale

# Create node connections
configIn = pipeline.create(dai.node.XLinkIn)
videoOut = pipeline.create(dai.node.XLinkOut)

# Stream names
configIn.setStreamName('config')
videoOut.setStreamName("video")

# Linking nodes
configIn.out.link(cam.inputConfig)
cam.video.link(manip.inputImage)
manip.out.link(videoOut.input)

with dai.Device(pipeline) as device:
    configQueue = device.getInputQueue('config')
    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

    while True:
        if video.has():
            frame = video.get().getCvFrame()
            cv2.imshow("video", frame if len(frame.shape) == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))  # Ensure greyscale

        if cv2.waitKey(1) == ord('q'):
            break
