# Description: This script publishes messages to a MQTT broker.

import paho.mqtt.client as mqtt
import keyboard
import time
import json

# Define MQTT broker details
broker_address = "localhost"
broker_port = 1883

# Define topic and message
topic = "optisort/scara/target"



# Create MQTT client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)

# Connect to MQTT broker
client.connect(broker_address, broker_port)
print("Connected to MQTT broker")
print("Press 'q' to quit")

i = 1
j = 0


def create_message(values):
    message_dict = {
        "x": values[0],
        "y": values[1],
        "z": values[2],
        "rx": values[3],
        "ry": values[4],
        "rz": values[5]
    }
    return json.dumps(message_dict)

time.sleep(3)

while True:

    match i:
        case 1:
            message = [425.00, 120.00, 340.00, 0.0, 180.0, -145.0]

        case 2:
            message = [375.00, 290.00, 340.00, 0.0, 180.0, -145.0]

        case 3:
            message = [375.00, 290.00, 340.00, 0.0, 180.0, -145.0]
            # message = [200.00, -430.00, 265.00, 0.0, 180.0, -145.0] # WARNING: this is very far and the robot hits flexibowl if is launched with the same approach


    i += 1
    if i == 4:
        i = 1

    json_message = create_message(message)

    # Publish message to topic
    client.publish(topic, str(json_message))
    print("Message sent: ")

    time.sleep(2)
    # Wait for user input to quit
    if keyboard.is_pressed('q'):
        print("\nqQuitting...")
        break

# Disconnect from MQTT broker
client.disconnect()
