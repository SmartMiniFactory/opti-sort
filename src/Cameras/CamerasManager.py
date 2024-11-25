import paho.mqtt.client as mqtt
import cv2
import json
import base64
import datetime
import time
from pyueye import ueye
from pypylon import pylon
import depthai as dai
import keyboard
import numpy as np

# MQTT
MQTT_BROKER = "localhost"  # IP address of the MQTT broker
MQTT_PORT = 1883  # Port of the MQTT Broker
MQTT_IDS_TOPIC = "optisort/ids/stream"  # Topic on which frame will be published
MQTT_LUXONIS_TOPIC = "optisort/luxonis/stream"  # Topic on which frame will be published
MQTT_BASLER_TOPIC = "optisort/basler/stream"  # Topic on which frame will be published

client = mqtt.Client()  # Create the MQTT Client
client.connect(MQTT_BROKER, MQTT_PORT)  # Establishing Connection with the Broker

# OpenCV
encode_param_png = [cv2.IMWRITE_PNG_COMPRESSION, 0]
encode_param_jpg = [int(cv2.IMWRITE_JPEG_QUALITY), 90]


# Image to JSON
def im2json(imdata):
    jstr = json.dumps(
        {"image": base64.b64encode(imdata).decode('ascii'), "timestamp": datetime.datetime.now().isoformat()})
    return jstr


def calculateFps(time):
    if time != 0:
        fps = 1 / time
    else:
        fps = 0
    return fps


# IDS ------------------------------------------------------------------------------------------------
# Connect to the first available camera and get its infos
hCam = ueye.HIDS(0)
sInfo = ueye.SENSORINFO()
cInfo = ueye.CAMINFO()
pcImageMemory = ueye.c_mem_p()
MemID = ueye.int()
rectAOI = ueye.IS_RECT()
pitch = ueye.INT()
nBitsPerPixel = ueye.INT(24)  # 24: bits per pixel for color mode; take 8 bits per pixel for monochrome
channels = 3  # 3: channels for color mode(RGB); take 1 channel for monochrome
m_nColorMode = ueye.INT()  # Y8/RGB16/RGB24/REG32
bytes_per_pixel = int(nBitsPerPixel / 8)

# Starts the driver and establishes the connection to the camera
nRet = ueye.is_InitCamera(hCam, None)
if nRet != ueye.IS_SUCCESS:
    print("is_InitCamera ERROR")

# Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
nRet = ueye.is_GetCameraInfo(hCam, cInfo)
if nRet != ueye.IS_SUCCESS:
    print("is_GetCameraInfo ERROR")

# You can query additional information about the sensor type used in the camera
nRet = ueye.is_GetSensorInfo(hCam, sInfo)
if nRet != ueye.IS_SUCCESS:
    print("is_GetSensorInfo ERROR")

nRet = ueye.is_ResetToDefault(hCam)
if nRet != ueye.IS_SUCCESS:
    print("is_ResetToDefault ERROR")

# Set display mode to DIB
nRet = ueye.is_SetDisplayMode(hCam, ueye.IS_SET_DM_DIB)

# Set the right color mode
if int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_BAYER:
    # setup the color depth to the current windows setting
    ueye.is_GetColorDepth(hCam, nBitsPerPixel, m_nColorMode)
    bytes_per_pixel = int(nBitsPerPixel / 8)

elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_CBYCRY:
    # for color camera models use RGB32 mode
    m_nColorMode = ueye.IS_CM_BGRA8_PACKED
    nBitsPerPixel = ueye.INT(32)
    bytes_per_pixel = int(nBitsPerPixel / 8)

elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_MONOCHROME:
    # for color camera models use RGB32 mode
    m_nColorMode = ueye.IS_CM_MONO8
    nBitsPerPixel = ueye.INT(8)
    bytes_per_pixel = int(nBitsPerPixel / 8)

else:
    # for monochrome camera models use Y8 mode
    m_nColorMode = ueye.IS_CM_MONO8
    nBitsPerPixel = ueye.INT(8)
    bytes_per_pixel = int(nBitsPerPixel / 8)

# Can be used to set the size and position of an "area of interest"(AOI) within an image
nRet = ueye.is_AOI(hCam, ueye.IS_AOI_IMAGE_GET_AOI, rectAOI, ueye.sizeof(rectAOI))
if nRet != ueye.IS_SUCCESS:
    print("is_AOI ERROR")

width = rectAOI.s32Width
height = rectAOI.s32Height

# Allocates an image memory for an image having its dimensions defined by width and height and its color depth defined by nBitsPerPixel
nRet = ueye.is_AllocImageMem(hCam, width, height, nBitsPerPixel, pcImageMemory, MemID)
if nRet != ueye.IS_SUCCESS:
    print("is_AllocImageMem ERROR")
else:
    # Makes the specified image memory the active memory
    nRet = ueye.is_SetImageMem(hCam, pcImageMemory, MemID)
    if nRet != ueye.IS_SUCCESS:
        print("is_SetImageMem ERROR")
    else:
        # Set the desired color mode
        nRet = ueye.is_SetColorMode(hCam, m_nColorMode)

# Activates the camera's live video mode (free run mode)
nRet = ueye.is_CaptureVideo(hCam, ueye.IS_DONT_WAIT)
if nRet != ueye.IS_SUCCESS:
    print("is_CaptureVideo ERROR")

# Enables the queue mode for existing image memory sequences
nRet = ueye.is_InquireImageMem(hCam, pcImageMemory, MemID, width, height, nBitsPerPixel, pitch)
if nRet != ueye.IS_SUCCESS:
    print("is_InquireImageMem ERROR")

# BASLER ------------------------------------------------------------------------------------------------
# Connecting to the first Basler available camera
baslerCamera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabbing Continuously (video) with minimal delay
baslerCamera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
baslerConverter = pylon.ImageFormatConverter()

# Converting to opencv bgr format
baslerConverter.OutputPixelFormat = pylon.PixelType_BGR8packed
baslerConverter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# LUXONIS ------------------------------------------------------------------------------------------------
pipeline = dai.Pipeline()# Create pipeline

camRgb = pipeline.create(dai.node.ColorCamera) # Define source
xoutRgb = pipeline.create(dai.node.XLinkOut) # Define output
xoutRgb.setStreamName("rgb")

camRgb.setPreviewSize(640, 480)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

camRgb.preview.link(xoutRgb.input)

# STREAMING  ------------------------------------------------------------------------------------------------
loopingCondition = True
keyPressed = False

with dai.Device(pipeline) as device:

    qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

    while loopingCondition:
        # IDS
        start = time.time()  # Start time
        array = ueye.get_data(pcImageMemory, width, height, nBitsPerPixel, pitch, copy=False)  # extracting data from camera
        frame = np.reshape(array, (height.value, width.value, bytes_per_pixel))  # reshaping with np
        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)  # Resize the image by a half
        _frame = cv2.imencode('.png', frame, encode_param_png)[1].tobytes()  # Encode image to PNG format
        client.publish(MQTT_IDS_TOPIC, im2json(_frame))  # Publish the Frame on the Topic
        cv2.imshow("IDS Camera Stream", frame)  # Show the frame
        end = time.time()  # End time
        t = end - start
        fps = calculateFps(t)
        print("FPS IDS: ", np.round(fps, 0), end="\r")

        # BASLER
        start = time.time()  # restart time
        grabResult = baslerCamera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if grabResult.GrabSucceeded():
            image = baslerConverter.Convert(grabResult)  # Access the image data
            img = image.GetArray()
            frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # Create an OpenCV frame
            frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        grabResult.Release()
        _frame = cv2.imencode('.png', frame, encode_param_png)[1].tobytes()
        client.publish(MQTT_BASLER_TOPIC, im2json(_frame))
        cv2.imshow("Basler Camera Stream", frame)
        end = time.time()  # End time
        t = end - start
        fps = calculateFps(t)
        print("FPS BASLER: ", np.round(fps, 0), end="\r")


        # LUXONIS
        inRgb = qRgb.get()  # blocking call, will wait until a new data has arrived
        start = time.time()
        frame = inRgb.getCvFrame()
        _frame = cv2.imencode('.png', frame, encode_param_png)[1].tobytes()
        client.publish(MQTT_LUXONIS_TOPIC, _frame)  # Publish the Frame on the Topic home/server
        cv2.imshow("Stream input", frame)  # Show the frame
        end = time.time()  # End time
        t = end - start
        fps = calculateFps(t)
        print("FPS LUXONIS: ", np.round(fps, 0), end="\r")  # Print the FPS


        # Press q if you want to end the loop
        if cv2.waitKey(1) & keyboard.is_pressed('q'):
            keyPressed = True

        loopingCondition = not keyPressed and (nRet == ueye.IS_SUCCESS)

# CLOSING SESSION  ------------------------------------------------------------------------------------------------

ueye.is_FreeImageMem(hCam, pcImageMemory, MemID)  # Releasing image memory allocated and removing from driver management
ueye.is_ExitCamera(hCam)  # Releases camera handle, releases, data structures, memory areas taken up by the camera

baslerCamera.StopGrabbing()

cv2.destroyAllWindows()
client.disconnect()
