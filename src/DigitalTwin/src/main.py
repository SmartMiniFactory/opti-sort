import sys
import os
import json
import time
import paho.mqtt.client as mqtt
from utils.MQTT_broker_address import MQTT_broker_data
import requests
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))


# Define an HTML session
session = requests.Session()

class IPhysics ():
    def __init__(self):
        self.var1 = 9
        self.client = mqtt.Client()            #create new instance
        self.client.on_connect= self.on_connect  #bind call back function
        self.client.on_message= self.on_message
        self.client.connect(MQTT_broker_data.HOST, MQTT_broker_data.PORT)               #connect to broker
        self.client.subscribe("DT_BROADCAST")
        self.client.loop_start()  #Start loop


    # MQTT_________________________________________________
    def on_connect(self, client, userdata, flags, reason_code):
        if reason_code == 0:
            print("Connected to the broker")
        if reason_code > 0:
            if reason_code == "Unsupported protocol version":
                print("Error: unsupported protocol version")

            if reason_code == "Client identifier not valid":
                print("Error: client identifier not valid")

        url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/SetJoint'
        # Define the headers
        headers = {'Content-Type': 'application/json'}

        # Define the body
        body = {"value": 1}




    def on_message(self, client, userdata, message):
        # This function is called every time a message is received on the subscribed topic

        # Convert the JSON message back to a Python dictionary
        msg = json.loads(message.payload)

        #  Filter only for messages directed to I-Physics
        if msg["receiver"] == "IPHYSICS":
            # assign codes to IPHYSICS commands
            if msg["command"] == 10:  # Update Scara joint values

                JOINT_POSITIONS = [msg["payload"]["q1"],
                                  msg["payload"]["q2"],
                                  msg["payload"]["q3"],
                                  msg["payload"]["q4"]]
                print("\rQ1: ", JOINT_POSITIONS[0], "Q2: ", JOINT_POSITIONS[1], "Q3: ", JOINT_POSITIONS[2], "Q4: ",
                      JOINT_POSITIONS[3], end="")
                #self.update_joint_positions(JOINT_POSITIONS)

    def update_joint_positions(self, JOINT_POSITIONS):
        print("\rQ1: ", JOINT_POSITIONS[0], "Q2: ", JOINT_POSITIONS[1], "Q3: ", JOINT_POSITIONS[2], "Q4: ", JOINT_POSITIONS[3], end="")

        #for i in range(4):
        #    self.send_joint_to_iphysics(i, JOINT_POSITIONS[i])  # Update the position in IPhysics

    def send_joint_to_iphysics(self, JOINT_ID, VAL):

        if JOINT_ID == 0:
            #url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/INNER LINK ASSY-1/KinActual'  # Depends on the IPhysics variable
            #url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/INNER LINK ASSY-1/Pippo'
            url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ1'

            # Define the headers
            headers = {'Content-Type': 'application/json'}

            # Define the body
            body = {"value": VAL}

        elif JOINT_ID == 1:
            url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ2'  # Depends on the IPhysics variable

            # Define the headers
            headers = {'Content-Type': 'application/json'}

            # Define the body
            body = {"value": np.deg2rad(VAL)}


        elif JOINT_ID == 2:
            url =f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ3'  # Depends on the IPhysics variable

            # Define the headers
            headers = {'Content-Type': 'application/json'}

            # Define the body
            body = {"value": VAL}

        elif JOINT_ID == 3:
            url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ4'  # Depends on the IPhysics variable

            # Define the headers
            headers = {'Content-Type': 'application/json'}

            # Define the body
            body = {"value": VAL}
            #print(f"Joints: value updated")

        else:
            url = ''
            headers = {}
            body = {}

        # Make the PUT request

        response = session.put(url=url, headers=headers, json=body)

        # Process the response
        if response.status_code == 200:

            return True
        else:
            # Handle error cases
            print(f"Joint {JOINT_ID}: error in updating the value. Error code: {response.status_code}")
            return False


if __name__ == '__main__':
    IPH = IPhysics()

    while True:
        pass