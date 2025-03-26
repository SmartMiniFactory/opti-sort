import sys
import os
import threading
import paho.mqtt.client as mqtt
import datetime
import time
import pathlib
import base64
import json
import xml.etree.ElementTree as ET
from transitions import Machine
import cv2
# from cameras.ids import Ids
# from cameras.basler import Basler
# from cameras.luxonis import Luxonis

# Get script details and localization
script_dir = pathlib.Path(__file__).parent.resolve()
script_name = pathlib.Path(__file__).name
temp_folder = script_dir / "../../OptiSort/HMI/Temp"
config_folder = script_dir / "../../OptiSort/HMI/Config"
script_id = str(os.getpid())


def publish(message, result):
    global script_dir
    if result is None:
        payload = {"script": {"path": (script_dir / script_name).as_posix(), "PID": script_id}, "message": message}
    else:
        payload = {"script": {"path": (script_dir / script_name).as_posix(), "PID": script_id}, "message": message, "result": result}
    mqttc.publish('optisort/camera_manager/output', str(json.dumps(payload)))


def on_publish(client, userdata, mid):
    print(f"Message Published: {client}, {userdata}, {mid}")


# MQTT SETUP
broker = '127.0.0.1'
port = 1883
client_name = 'camera_manager'
MQTT_KEEPALIVE_INTERVAL = 60
mqttc = mqtt.Client()  # Initiate MQTT Client

def parse_appconfig(file_path):
    camera_topics = []

    # parse appconfig XML
    tree = ET.parse(file_path)
    for element in tree.getroot():
        if element.tag == "userSettings":
            for setting in element[0]:

                if setting.attrib['name'] == 'mqtt_broker':
                    broker = setting[0].text

                if setting.attrib['name'] == 'mqtt_topic_idsStream':
                    camera_topics.append(setting[0].text)

                if setting.attrib['name'] == 'mqtt_topic_luxonisStream':
                    camera_topics.append(setting[0].text)

                if setting.attrib['name'] == 'mqtt_topic_baslerStream':
                    camera_topics.append(setting[0].text)

                if setting.attrib['name'] == 'mqtt_port':
                    port = int(setting[0].text)

        return broker, port, camera_topics


def im2json(imdata):
    jstr = json.dumps(
        {"image": base64.b64encode(imdata).decode('ascii'), "timestamp": datetime.datetime.now().isoformat()})
    return jstr

# Define states
"""
init: distinction between test and real camera; connection with camera and resource lock
idle: waiting for configuration command; landing point after camera activity completes
config: camera setup that depends on the specific need, next state depends on mqtt parameters
...
"""
states = ['init', 'webcam', 'cameras', 'idle', 'config', 'ready', 'streaming', 'processing', 'ended']

# State Machine Class
class StateMachine:
    def __init__(self):
        self.machine = Machine(model=self, states=states, initial='init')
        self.machine.add_transition('initialize', 'init', 'idle', after=self.idle)
        self.machine.add_transition('set_idle', '*', 'idle', after=self.idle)
        self.machine.add_transition('configure', 'idle', 'ready', after=self.ready)
        self.machine.add_transition('start_streaming', 'ready', 'streaming', after=self.stream)
        self.machine.add_transition('start_processing', 'ready', 'processing', after=self.process)
        self.machine.add_transition('terminate', '*', 'ended', after=self.exit_script)

        # flags
        self.testing = None # should become true or false
        self.streaming = False
        self.processing = False
        self.target_camera = None

        self.webcam = None
        self.ids = None
        self.basler = None
        self.luxonis = None
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

    def on_message(self, client, userdata, msg):
        print(f"Received message on {msg.topic}: {msg.payload.decode('utf-8')}", None)
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            command = payload.get("command")
            # print(f"Parsed command: {command}")  # Debugging line

            if self.state == 'init':

                if command == "webcam":
                    self.testing = True
                    self.initialize()

                elif command == "cameras":
                    self.testing = False
                    self.initialize()

                else:
                    publish("You should set webcam or cameras first", None)

            else:
                if command == "streaming":
                    self.streaming = True
                    self.configure()

                elif command == "processing":
                    self.processing = True
                    self.configure()

                elif command == "start" and self.streaming:
                    self.start_streaming()

                elif command == "start" and self.processing:
                    self.start_processing()

                elif command == "stop":
                    self.set_idle()

                elif command == "exit":
                    publish("terminating program", None)
                    self.terminate()

                else:
                    publish("Command not recognized", None)

            # print(f"State changed to: {self.state}")  # Debugging line

        except Exception as e:
            print(f"Error processing message: {e}")


    def initialization(self):

        publish("I'm initializing", None)

        if self.testing:
            self.webcam = cv2.VideoCapture(0)

        else:
            # self.ids = Ids(camera_id="ids")
            self.ids.initialize()
            publish("IDS camera initialized", None)

            # self.basler = Basler(camera_id="basler")
            self.basler.initialize()
            publish("Basler camera initalized", None)

            # self.luxonis = Luxonis(camera_id="luxonis")
            self.luxonis.initialize()
            publish("Luxonis camera initialized", None)

        self.set_idle()


    def idle(self):
        publish("Cameras initialized; waiting for config parameters", None)
        self.streaming = False
        self.processing = False

    def configuration(self):

        ids_configfile = (script_dir / "../../OptiSort/HMI/Config/camera_configuration.ini").resolve()
        basler_configfile = (script_dir / "../../OptiSort/HMI/Config/camera_config.pfs").resolve()

        publish("I'm configuring", None)


    def ready(self):
        publish("Cameras configured; send start command", None)


    def process(self):
        publish("I'm processing", None)

    def stream(self):
        publish("I'm streaming", None)


    def exit_script(self):
        # TODO: release resources
        publish("Exiting state machine and releasing resources...", None)
        sys.exit(0)


def main():
    mqttc.on_publish = on_publish  # Register publish callback function
    mqttc.connect(broker, port, MQTT_KEEPALIVE_INTERVAL)  # Connect with MQTT Broker
    mqttc.subscribe("optisort/camera_manager/input")

    publish("Camera manager script started and connected to MQTT broker", None)

    state_machine = StateMachine()
    mqttc.on_message = state_machine.on_message
    mqttc.loop_forever()


if __name__ == "__main__":
    main()