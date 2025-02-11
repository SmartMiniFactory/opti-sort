import paho.mqtt.client as mqtt
from utils.MQTT_broker_address import MQTT_broker_data
import json
from queue import Queue
import threading
import requests
import numpy as np

# Define an HTML session
session = requests.Session()

# Create 4 queues to store the joint values
queue_q1 = Queue()
queue_q2 = Queue()
queue_q3 = Queue()
queue_q4 = Queue()

# Define the URLs for the PUT requests
url_q1 = f"http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ1"
url_q2 = f"http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ2"
url_q3 = f"http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ3"
url_q4 = f"http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ4"


def publish_html_put(queue, url, body=None, headers=None):
    while True:
        if not queue.empty():
            # Define the headers
            headers = {'Content-Type': 'application/json'}

            # Define the body
            body = {"value": queue.get()}

            # Make the PUT request
            response = session.put(url=url, headers=headers, json=body)

            # Process the response
            if response.status_code == 200:
                return True
            else:
                # Handle error cases
                return False


# Define the callback functions
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected with result code {rc}")


def on_message(client, userdata, msg):
    # This function is called every time a message is received on the subscribed topic
    # Convert the JSON message back to a Python dictionary
    msg = json.loads(msg.payload)

    #  Filter only for messages directed to I-Physics
    if msg["receiver"] == "IPHYSICS":
        # assign codes to IPHYSICS commands
        if msg["command"] == 10:  # Update Scara joint values
            queue_q1.put(np.deg2rad(msg["payload"]["q1"]))
            queue_q2.put(np.deg2rad(msg["payload"]["q2"]))
            queue_q3.put(msg["payload"]["q3"])
            queue_q4.put(np.deg2rad(msg["payload"]["q4"]))


# Create an MQTT client instance
client = mqtt.Client()

# Assign the callback functions
client.on_connect = on_connect
client.on_message = on_message

# Connect to the MQTT broker
client.connect(MQTT_broker_data.HOST, MQTT_broker_data.PORT, 60)
client.subscribe("DT_BROADCAST")

# Start the loop
client.loop_start()

# Create and start the threads
thread_q1 = threading.Thread(target=publish_html_put, args=(queue_q1, url_q1))
thread_q2 = threading.Thread(target=publish_html_put, args=(queue_q2, url_q2))
thread_q3 = threading.Thread(target=publish_html_put, args=(queue_q3, url_q3))
thread_q4 = threading.Thread(target=publish_html_put, args=(queue_q4, url_q4))

thread_q1.start()
thread_q2.start()
thread_q3.start()
thread_q4.start()

while True:
    # Keep the main thread alive
    thread_q1.join()
    thread_q2.join()
    thread_q3.join()
    thread_q4.join()