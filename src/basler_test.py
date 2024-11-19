####################################################### LIBRARIES #######################################################
import cv2
import keyboard
import paho.mqtt.client as mqtt
import base64
import time
import numpy as np
from pypylon import pylon
import json
import datetime

####################################################### VARIABLES #######################################################

# MQTT
MQTT_BROKER = "localhost" # IP address of the MQTTT broker
MQTT_PORT = 1883 # Port of the MQTT Broker
MQTT_TOPIC = "optisort/basler/stream" # Topic on which frame will be published

# OpenCV
encode_param_png = [cv2.IMWRITE_PNG_COMPRESSION, 0]
encode_param_jpg = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

########################################### USER-DEFINED FUNCTIONS ######################################################


################################################## MAIN PROGRAM #########################################################

print("Connecting to MQTT broker")
client = mqtt.Client() # Create the MQTT Client
client.connect(MQTT_BROKER, MQTT_PORT) # Establishing Connection with the Broker
print("Connected to MQTT broker")

print("Connecting to the Basler camera")

# Conecting to the first Basler available camera
baslerCamera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabing Continusely (video) with minimal delay
baslerCamera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly) 
baslerConverter = pylon.ImageFormatConverter()

# Converting to opencv bgr format
baslerConverter.OutputPixelFormat = pylon.PixelType_BGR8packed
baslerConverter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

if not baslerCamera.IsGrabbing():
    print("Error: Could not open camera.")
print("Connected to Basler camera")

print("Start streaming")
print("Press 'Q' to quit")

# Encode image to Json format
def im2json(imdata):
    jstr = json.dumps({"image": base64.b64encode(imdata).decode('ascii'), "timestamp": datetime.datetime.now().isoformat()})
    return jstr

while True:

    start = time.time() # Start time

    grabResult = baslerCamera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        # Access the image data
        image = baslerConverter.Convert(grabResult)
        img = image.GetArray()
        
        # Create an OpenCV frame
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Resize the image by a half
        frame = cv2.resize(frame,(0,0),fx=0.5, fy=0.5)
  
    grabResult.Release()
    

    # Encode image to PNG format
    _frame = cv2.imencode('.png', frame, encode_param_png)[1].tobytes()
    
    # Publish the Frame on the Topic
    client.publish(MQTT_TOPIC, im2json(_frame))
    
    # Show the frame
    cv2.imshow("Basler Camera Stream", frame)
    end = time.time() # End time
    t = end - start
    fps = 1/t

    print("FPS Basler: ", np.round(fps, 0), end="\r") # Print the FPS

    
    # Press q if you want to end the loop
    if cv2.waitKey(1) & keyboard.is_pressed('q'):
        print("\nQuitting...")
        break

# Releasing the resource    
baslerCamera.StopGrabbing()
cv2.destroyAllWindows()

client.disconnect()
print("\nStopped streaming")