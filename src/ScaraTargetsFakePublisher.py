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
message = [300.07, -0.959, 302.12, 0.0, 180.0, 180.0]


# Create MQTT client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)

# Connect to MQTT broker
client.connect(broker_address, broker_port)
print("Connected to MQTT broker")
print("Press 'q' to quit")

i = 0
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


while True:

    match i:
        case 1:
            message = [300.07, -0.959, 302.12, 0.0, 180.0, 180.0]

        case 2:
            message = [250.07, -50.959, 302.12, 0.0, 180.0, 180.0]

        case 3:
            message = [200.07, -50.959, 302.12, 0.0, 180.0, 180.0]

    i += 1
    if i == 4:
        i = 0

    json_message = create_message(message)

    # Publish message to topic
    client.publish(topic, str(json_message))
    print("Message sent: ")

    time.sleep(1)
    # Wait for user input to quit
    if keyboard.is_pressed('q'):
        print("\nqQuitting...")
        break

# Disconnect from MQTT broker
client.disconnect()
