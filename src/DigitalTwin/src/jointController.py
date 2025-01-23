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
JOINT_CONTROL = False

# Define an HTML session
session = requests.Session()

class IPhysics ():

    def set_joint_control(self, VAL):
        url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/SetJoint'
        # Define the headers
        headers = {'Content-Type': 'application/json'}

        # Define the body
        body = {"value": VAL}

        # Make the PUT request

        response = session.put(url=url, headers=headers, json=body)

        # Process the response
        if response.status_code == 200:
            print("Joint control enabled in IPhysics")
            return True
        else:
            # Handle error cases
            print(f"Error code: {response.status_code}")
            return False

    def send_joint_to_iphysics(self, JOINT_ID, VAL):

        if JOINT_ID == 0:
            #url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/INNER LINK ASSY-1/KinActual'  # Depends on the IPhysics variable
            #url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/INNER LINK ASSY-1/Pippo'
            url = f'http://localhost:8080/io/COBRA 600 CAD MODEL/BASE ASSY-1/tQ1'

            # Define the headers
            headers = {'Content-Type': 'application/json'}

            # Define the body
            body = {"value": np.deg2rad(VAL)}

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
            body = {"value": np.deg2rad(VAL)}
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
    JOINT_CONTROL = True# IPH.set_joint_control(1)
    x = -100
    while JOINT_CONTROL:
        for i in range(50):
            x = x + 1
            JOINT_POSITIONS = [x,30,10,5]
            print(JOINT_POSITIONS)
            for j in range(4):
                IPH.send_joint_to_iphysics(j, JOINT_POSITIONS[j])  # Update the position in IPhysics
            time.sleep(0.1)
        break
    print("Cycle ended")