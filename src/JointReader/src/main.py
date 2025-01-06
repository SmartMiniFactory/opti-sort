import sys
import os
import json
import time
import paho.mqtt.client as mqtt
from utils.MQTT_broker_address import MQTT_broker_data

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))


class IPhysics ():
    def __init__(self):
        self.var1 = 9
        self.client = mqtt.Client()             #create new instance
        self.client.on_connect= self.on_connect  #bind call back function
        self.client.on_message= self.on_message
        self.client.connect(MQTT_broker_data.HOST, MQTT_broker_data.PORT)               #connect to broker
        self.client.subscribe("DT_BROADCAST")
        self.client.loop_start()  #Start loop

    # MQTT_________________________________________________
    def on_connect(self, client, userdata, flags, rc):
        print(rc)
        if rc==0:
            print("connected ok")

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

                self.update_joint_positions(JOINT_POSITIONS)

    def update_joint_positions(self, JOINT_POSITIONS):
        print("\rQ1: ", JOINT_POSITIONS[0],"Q2: ", JOINT_POSITIONS[1],"Q3: ", JOINT_POSITIONS[2],"Q4: ", JOINT_POSITIONS[3], end="")

if __name__ == '__main__':
    IPH = IPhysics()

    while True:
        time.sleep(2)