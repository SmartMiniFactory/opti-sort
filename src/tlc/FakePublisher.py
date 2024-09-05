# Description: This script publishes messages to a MQTT broker.

import paho.mqtt.client as mqtt
import keyboard
import time

# Define MQTT broker details
broker_address = "localhost"
broker_port = 1883

# Define topic and message
topic = "optisort/scara/target"
message = [300.07, -0.959, 302.12, 0.0, 180.0, 180.0]
#message = "{'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}"

# Create MQTT client
client = mqtt.Client()

# Connect to MQTT broker
client.connect(broker_address, broker_port)
print("Connected to MQTT broker")
print("Press 'q' to quit")

i = 0
j=0

while True:
    if i >= 0 and i < 200:
        message = [300.07, -0.959, 302.12, 0.0, 180.0, 180.0]
        
        i += 1
    elif i >= 200 and i < 400:
        message = [250.07, -50.959, 302.12, 0.0, 180.0, 180.0]
        i += 1
    elif i >= 400 and i < 600:
        message = [200.07, -50.959, 302.12, 0.0, 180.0, 180.0]
        i +=1
    else:
        i=0

    # Publish message to topic
    client.publish(topic, str(message))
    print("Message sent: " + str(message), end = "\r")
    #qprint("Message sent: " + str(message))
    time.sleep(0.01)
    # Wait for user input to quit
    if keyboard.is_pressed('q'):
        print("\nqQuitting...")
        break

# Disconnect from MQTT broker
client.disconnect()