"""
Description: This script publishes messages to a MQTT broker.
The published message represent a set of fake coordinates as one of the cameras were detecting some object
"""
import paho.mqtt.client as mqtt
import keyboard
import time
import json
import os
import pathlib


# Get script details and localization
script_dir = pathlib.Path(__file__).parent.resolve()
script_name = pathlib.Path(__file__).name
temp_folder = script_dir / "../../OptiSort/HMI/Temp"
config_folder = script_dir / "../../OptiSort/HMI/Config"
script_id = str(os.getpid())

# MQTT SETTINGS
broker = '127.0.0.1'
port = 1883
client_name = 'fake_process'
MQTT_KEEPALIVE_INTERVAL = 60
mqttc = mqtt.Client()  # Initiate MQTT Client

def publish(message, result):

    global script_dir
    payload = {
        "script": {
            "path": (script_dir / script_name).as_posix(),
            "PID": script_id
        },
        "message": message
    }
    if result is not None:
        payload["result"] = result

    mqttc.publish('optisort/scara/target', str(json.dumps(payload)), qos=0)

message = [300.07, -0.959, 302.12, 0.0, 180.0, 180.0]

# Connect to MQTT broker
mqttc.connect(broker, port, MQTT_KEEPALIVE_INTERVAL)  # Connect with MQTT Broker
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
    return message_dict


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
    publish(json_message, None)
    print("Message sent: ")

    time.sleep(1)
    # Wait for user input to quit
    if keyboard.is_pressed('q'):
        print("\nqQuitting...")
        break

# Disconnect from MQTT broker
mqttc.disconnect()