import queue
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


# publishes its own messages on optisort/camera_manager/output
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

    mqttc.publish('optisort/camera_manager/output', str(json.dumps(payload)), qos=0)


# informs the developer via console about having published a message over MQTT (debug)
def on_publish(client, userdata, mid):
    print(f"Message Published: {client}, {userdata}, {mid}")

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        publish("Camera manager online and connected to MQTT", None)  # publish when connection is ensured
    else:
        print(f"Failed to connect, return code {rc}")


# MQTT SETTINGS
broker = '127.0.0.1'
port = 1883
client_name = 'camera_manager'
MQTT_KEEPALIVE_INTERVAL = 60
mqttc = mqtt.Client()  # Initiate MQTT Client


# Reads settings of the OPTISORT solutions to use the same topics (user may change those)
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


# Converts images (base 64) to JSON
def im2json(imdata):
    jstr = json.dumps(
        {"image": base64.b64encode(imdata).decode('ascii'), "timestamp": datetime.datetime.now().isoformat()})
    return jstr


class CameraManager:
    def __init__(self, testing=False):
        self.testing = testing
        self.cameras = {}
        self.lock = threading.Lock()

        if testing:
            try:
                self.cameras['webcam'] = cv2.VideoCapture(0)
                publish("Webcam initialized", None)

            except Exception as e:
                publish(f"No available webcam found: {e}", None)

        else:
            self.cameras['ids'] = Ids(camera_id="ids")
            self.cameras['ids'].initialize()
            publish("IDS camera initialized", None)

            self.cameras['basler'] = Basler(camera_id="basler")
            self.cameras['basler'].initialize()
            publish("Basler camera initalized", None)

            self.cameras['luxonis'] = Luxonis(camera_id="luxonis")
            self.cameras['luxonis'].initialize()
            publish("Luxonis camera initialized", None)


    def configure(self, target_camera):
        if not self.testing:
            ids_configfile = (script_dir / "../../OptiSort/HMI/Config/camera_configuration.ini").resolve()
            basler_configfile = (script_dir / "../../OptiSort/HMI/Config/camera_config.pfs").resolve()

            # TODO: create configure_freerun and configure_triggered in base camera

            try:
                if target_camera is None:
                    # self.cameras['ids'].configure(ids_configfile)
                    self.cameras['basler'].configure(basler_configfile)
                    # self.cameras['luxonis'].configure(None)
                elif target_camera == 'ids':
                    self.cameras['ids'].configure(ids_configfile)
                elif target_camera == 'basler':
                    self.cameras['basler'].configure(basler_configfile)
                elif target_camera == 'luxonis':
                    aaa = True
                    # self.cameras['luxonis'].configure(None)
                else:
                    publish("target camera not recognized", None)

            except:
                publish("Cameras configuration failed", None)

    def start_acquisition(self, cam_name):
        self.cameras[cam_name].acquisition_start()

    def capture_frame(self, cam_name):
        with self.lock:

            if self.testing and cam_name == 'webcam':
                ret, frame = self.cameras['webcam'].read()
                return frame if ret else None

            elif cam_name in self.cameras:
                return self.cameras[cam_name].capture_frame()
        return None

    def release(self):
        for cam in self.cameras.values():
            if isinstance(cam, cv2.VideoCapture):
                cam.release()


class StreamingHandler:
    def __init__(self, camera_manager):
        self.camera_manager = camera_manager
        self.threads = {}
        self.running = threading.Event()

    def start(self):
        self.running.set()
        if self.camera_manager.testing:
            cameras = ['webcam', 'webcam', 'webcam']
        else:
            cameras = ['ids', 'basler', 'luxonis']

        for cam in cameras:
            self.threads[cam] = threading.Thread(target=self.stream_camera, args=(cam,))
            self.threads[cam].start()


    def stream_camera(self, cam_name):
        self.camera_manager.start_acquisition(cam_name)
        while self.running.is_set():

            # Calculate the next publication time (synchronized to a 1-second clock)
            next_publish_time = time.time() + 0.5

            frame = self.camera_manager.capture_frame(cam_name)
            if frame is not None:
                encoded = cv2.imencode(".png", frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])[1].tobytes()
                if encoded:
                    if cam_name == 'webcam':
                        mqttc.publish("optisort/ids/stream", im2json(encoded))
                        mqttc.publish("optisort/basler/stream", im2json(encoded))
                        mqttc.publish("optisort/luxonis/stream", im2json(encoded))
                    else:
                        mqttc.publish(f"optisort/{cam_name}/stream", im2json(encoded))

                        # Calculate sleep duration to sync to the clock
            now = time.time()
            sleep_duration = max(next_publish_time - now, 0)  # Avoid negative sleep
            time.sleep(sleep_duration)

    def stop(self):
        self.running.clear()
        for thread in self.threads.values():
            thread.join()


class ProcessingHandler(threading.Thread):
    def __init__(self, camera_manager, target_camera):
        super().__init__()
        self.camera_manager = camera_manager
        self.target_camera = target_camera
        self.queue = queue.Queue()
        self.running = threading.Event()
        self.running.set()

    def run(self):
        while self.running.is_set():
            frame = self.camera_manager.capture_frame(self.target_camera)
            if frame is not None:
                encoded, buffer = cv2.imencode('.jpg', frame)
                if encoded:
                    mqttc.publish(f"optisort/{self.target_camera}/stream", im2json(buffer))
            time.sleep(0.01)

    def stop(self):
        self.running.clear()


states = ['init', 'webcam', 'cameras', 'idle', 'config', 'ready', 'streaming', 'processing', 'ended']


# State Machine Class
class StateMachine:

    def __init__(self):
        self.machine = Machine(model=self, states=states, initial='init')
        self.machine.add_transition('initialize', 'init', 'idle', after=self.idle)
        self.machine.add_transition('set_idle', '*', 'idle', after=self.idle)
        self.machine.add_transition('configure', 'idle', 'ready', after=self.config)
        self.machine.add_transition('start_stream', 'ready', 'streaming', after=self.stream)
        self.machine.add_transition('start_process', 'ready', 'processing', after=self.process)
        self.machine.add_transition('terminate', '*', 'ended', after=self.exit_script)

        self.camera_manager = None
        self.streaming_handler = None
        self.processing_handler = None
        self.testing = False
        self.target_camera = None
        self.running = True

    def on_message(self, client, userdata, msg):
        print(f"Received message on {msg.topic}: {msg.payload.decode('utf-8')}, state: {self.state}")
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
            command = payload.get("command")

            # in the init state is mandatory to tell the machine about using webcams or cameras
            if self.state == 'init':
                if command != "webcam" and command != "cameras":
                    publish("Which optics do you want to use? [webcam, cameras]", None)
                else:
                    if command == "webcam":
                        publish("Initializing webcam...", None)
                        self.testing = True
                    elif command == "cameras":
                        publish("Initializing cameras...", None)
                        self.testing = False

                    self.camera_manager = CameraManager(self.testing)
                    self.initialize()

            else:

                if command == "streaming":
                    publish("Configuring streaming parameters...", None)
                    self.configure()

                elif command == "processing":
                    self.target_camera = payload.get("camera")
                    if self.target_camera is None:
                        publish("Please specify a target camera: [ids, basler, luxonis]", None)
                    else:
                        publish("Configuring target camera processing parameters...", None)
                        self.configure()

                elif command == "start" and self.state == 'ready':
                    if self.target_camera is None:
                        publish("Starting streaming...", None)
                        self.start_stream()
                    else:
                        publish("Starting processing...", None)
                        self.start_process()


                elif command == "stop":
                    publish("Streaming/processing stopped!", None)
                    self.set_idle()

                elif command == "exit":
                    publish("Terminating program...", None)
                    self.terminate()

                else:
                    publish(f"Command '{command}' not recognized", None)

            # print(f"State changed to: {self.state}")  # Debugging line

        except Exception as e:
            print(f"Error processing message: {e}")

    def idle(self):
        self.target_camera = None
        publish("Cameras initialized! Send functioning mode [streaming, processing]", None)

    def config(self):
        self.camera_manager.configure(self.target_camera)
        publish("Cameras configured! Send start command", None)

    def stream(self):
        self.streaming_handler = StreamingHandler(self.camera_manager)
        self.streaming_handler.start()
        publish("Stream started!", None)

    def process(self):
        self.processing_handler = ProcessingHandler(self.camera_manager, self.target_camera)
        self.processing_handler.start()
        publish("Process started!", None)


    def exit_script(self):

        if self.streaming_handler:
            self.streaming_handler.stop()

        if self.processing_handler:
            self.processing_handler.stop()

        self.camera_manager.release()
        self.running = False


def main():
    mqttc.on_connect = on_connect  # Register connect callback
    mqttc.on_publish = on_publish  # Register publish callback function
    mqttc.connect(broker, port, MQTT_KEEPALIVE_INTERVAL)  # Connect with MQTT Broker
    mqttc.subscribe("optisort/camera_manager/input")  # subscribe to topic for receiving commands
    mqttc.loop_start()  # Start the loop in a separate thread

    state_machine = StateMachine()  # activate state machine class
    mqttc.on_message = state_machine.on_message  # attach MQTT messages to state machine class

    while state_machine.running:
        time.sleep(0.05)  # Prevents 100% CPU usage


    # Ensure cleanup
    mqttc.loop_stop()
    mqttc.disconnect()



if __name__ == "__main__":
    main()
