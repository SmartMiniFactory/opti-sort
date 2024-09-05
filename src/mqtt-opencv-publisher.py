####################################################### LIBRARIES #######################################################
import cv2
import keyboard
import paho.mqtt.client as mqtt
import base64
import time
import numpy as np

####################################################### VARIABLES #######################################################

# MQTT
BOKER_ID = 2
MQTT_BROKER = ["192.168.0.102", "localhost", "10.10.238.20"] # IP address of the MQTTT broker
MQTT_PORT = 1883 # Port of the MQTT Broker
MQTT_TOPIC = "optisort/luxonis/stream" # Topic on which frame will be published
MQTT_USER = "dgalli" # Username for the MQTT Broker
MQTT_PASSWORD = "dgalli" # Password for the MQTT Broker

# OpenCV
WEBCAM_ID = 0  # Change this to the index of the desired webcam
encode_param_png = [cv2.IMWRITE_PNG_COMPRESSION, 0]
encode_param_jpg = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

########################################### USER-DEFINED FUNCTIONS ######################################################


################################################## MAIN PROGRAM #########################################################

print("Connecting to MQTT broker")
client = mqtt.Client() # Create the MQTT Client
if BOKER_ID == 0: # If the borker is autorized, set username and password
    client.username_pw_set(MQTT_USER, MQTT_PASSWORD) # Setting username and password
client.connect(MQTT_BROKER[BOKER_ID], MQTT_PORT) # Establishing Connection with the Broker
print("Connected to MQTT broker")


print("Opening the camera")
stream = cv2.VideoCapture(WEBCAM_ID) # Object to capture the frames
if not stream.isOpened():
    print("Error: Could not open camera.")
    exit()
print("Starting streaming")
print("Press 'Q' to quit")

while True:
    start = time.time() # Start time
    _, frame = stream.read() # Read frame

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
        break

stream.release()
client.disconnect()
print("\nStopped streaming")