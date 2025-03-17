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
from cameras.ids import Ids
from cameras.basler import Basler
from cameras.luxonis import Luxonis

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
all_streaming_freerun: all three cameras stream at lowest possible quality (intended for supervision)
single_streaming_freerun: only the target camera is streaming at a fixed rate at maximum detail (intended for process)
single_streaming_triggered: only the target camera acquires frames based on mqtt triggers, at max detail (int. for process)
"""
states = ['init', 'idle', 'config', 'all_streaming_freerun', 'single_streaming_freerun', 'single_streaming_triggered', 'exiting']

# State Machine Class
class StateMachine:
    def __init__(self):
        self.machine = Machine(model=self, states=states, initial='init')

        self.machine.add_transition('set_idle', '*', 'idle', after=self.idle)
        self.machine.add_transition('configure', 'idle', 'streaming') # state depends on parameters
        self.machine.add_transition('freerun-all', 'config', 'all_streaming_freerun', after=self.streaming_A)
        self.machine.add_transition('freerun-single', 'config', 'single_streaming_freerun', after=self.streaming_B)
        self.machine.add_transition('freerun-triggered', 'config', 'single_streaming_triggered', after=self.streaming_C)
        self.machine.add_transition('terminate', 'idle', 'ended', after=self.exit_script())

        self.testing = None
        self.mode = None
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

            if command == "init":
                self.testing = payload.get("webcam")  # should be a boolean
                self.initialization()


            elif command == "config":
                if payload.get("mode") == "idle-streaming":
                    self.mode = 'A'
                    self.configure()

                elif payload.get("mode") == "simple-process":
                    self.mode = 'B'
                    self.configure()

                elif payload.get("mode") == "optimized-process":
                    self.mode = 'C'
                    self.configure()

                else:
                    publish("Unrecognized operational mode", None)


            elif command == "reset":
                self.reset()

            elif command == "exit":
                self.exit()

            # print(f"State changed to: {self.state}")  # Debugging line

        except Exception as e:
            print(f"Error processing message: {e}")


    def initialization(self):
        if self.testing:
            self.webcam = cv2.VideoCapture(0)

        else:
            self.ids = Ids(camera_id="ids")
            self.ids.initialize()
            publish("IDS camera initialized", None)

            self.basler = Basler(camera_id="basler")
            self.basler.initialize()
            publish("Basler camera initalized", None)

            self.luxonis = Luxonis(camera_id="luxonis")
            self.luxonis.initialize()
            publish("Luxonis camera initialized", None)

        self.set_idle()


    def idle(self):
        publish("Cameras initialized; waiting for config parameters", None)

    def configuration(self):

        ids_configfile = (script_dir / "../../OptiSort/HMI/Config/camera_configuration.ini").resolve()
        basler_configfile = (script_dir / "../../OptiSort/HMI/Config/camera_config.pfs").resolve()

        if self.mode == 'A':
            if not self.testing:
                self.ids.configure(ids_configfile)
                self.basler.configure(basler_configfile)
                self.luxonis.configure(None)
            self.streaming_A()

        elif self.mode == 'B':
            if self.target_camera == 'ids':
                self.ids.acquisition_start()
                self.streaming_B(self.ids, "optisort/ids/stream")

            elif self.target_camera == 'basler':
                self.basler.acquisition_start()
                self.streaming_B(self.basler, "optisort/basler/stream")

            elif self.target_camera == 'luxonis':
                self.luxonis.acquisition_start()
                self.streaming_B(self.luxonis, "optisort/luxonis/stream")

        elif self.mode == 'C':
            if self.target_camera == 'ids':
                self.streaming_C(self.ids)
            elif self.target_camera == 'basler':
                self.streaming_C(self.basler)
            elif self.target_camera == 'luxonis':
                self.streaming_C(self.luxonis)


    # async def streaming_A(self):

    async def streaming_B(self, target_camera, topic):
        while self.state == 'streaming':
            try:
                frame = target_camera.capture_frame()
                if frame is not None:
                    encoded_frame = cv2.imencode(".jpg", frame, self.encode_param)[1].tobytes()
                    mqttc.publish(topic, im2json(encoded_frame))

                await time.sleep(0.5)  # Set frame rate independently for each camera
            except Exception as e:
                print(f"Error capturing frame from {topic}: {e}")

    # async def streaming_C(self, target_camera):


    def exit_scritp(self):
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