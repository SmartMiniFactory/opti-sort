# === IMPORTS ===
import os
import threading
import time
import datetime
import pathlib
import base64
import json
import queue
import xml.etree.ElementTree as ET
import cv2
import paho.mqtt.client as mqtt
from transitions import Machine

# Import custom camera interfaces and image processor
from cameras.ids import Ids
from cameras.basler import Basler
from cameras.luxonis import Luxonis
from processing.image_processor import ImageProcessor

# === GLOBAL VARIABLES ===
# Define paths and script details
script_dir = pathlib.Path(__file__).parent.resolve()
script_name = pathlib.Path(__file__).name
script_id = str(os.getpid())
temp_folder = script_dir / "../../OptiSort/HMI/Temp"
config_folder = script_dir / "../../OptiSort/HMI/Config"
ids_configfile = config_folder / "ids_configuration.ini"
basler_configfile = config_folder / "basler_configuration.pfs"

# MQTT configuration
broker = '127.0.0.1'
port = 1883
client_name = 'camera_manager'
MQTT_KEEPALIVE_INTERVAL = 60
mqttc = mqtt.Client()


# === UTILITY FUNCTIONS ===

def publish(message, result=None):
    """Publishes a JSON-formatted message with script details over MQTT."""
    payload = {
        "script": {
            "path": (script_dir / script_name).as_posix(),
            "PID": script_id
        },
        "message": message
    }
    if result is not None:
        payload["result"] = result

    mqttc.publish('optisort/camera_manager/output', json.dumps(payload), qos=0)


def on_publish(client, userdata, mid):
    """Callback for confirming MQTT publish (debugging)."""
    print(f"Message Published: {client}, {userdata}, {mid}")


def on_connect(client, userdata, flags, rc):
    """Callback for MQTT connection event."""
    if rc == 0:
        print("Connected to MQTT Broker!")
        publish("Camera manager started. Select mode [webcam, cameras]", 0)
    else:
        print(f"Failed to connect, return code {rc}")


def parse_appconfig(file_path):
    """Parses XML configuration to extract broker, port, and camera topics."""
    camera_topics = []
    tree = ET.parse(file_path)
    for element in tree.getroot():
        if element.tag == "userSettings":
            for setting in element[0]:
                name = setting.attrib['name']
                value = setting[0].text
                if name == 'mqtt_broker':
                    global broker
                    broker = value
                elif name in ['mqtt_topic_idsStream', 'mqtt_topic_luxonisStream', 'mqtt_topic_baslerStream']:
                    camera_topics.append(value)
                elif name == 'mqtt_port':
                    global port
                    port = int(value)
    return broker, port, camera_topics


def im2json(imdata):
    """Encodes image bytes and timestamp as JSON."""
    return json.dumps({
        "image": base64.b64encode(imdata).decode('ascii'),
        "timestamp": datetime.datetime.now().isoformat()
    })


def check_file_readable(file_path):
    """Checks if file exists and is readable."""
    if not file_path.is_file():
        raise FileNotFoundError(f"File {file_path} does not exist.")
    if not os.access(file_path, os.R_OK):
        raise PermissionError(f"File {file_path} is not readable.")


# === CAMERA MANAGER ===

class CameraManager:
    """Manages initialization, configuration, and acquisition of cameras."""

    def __init__(self, testing=False):
        self.testing = testing
        self.cameras = {}
        self.lock = threading.Lock()

        if testing:
            self._initialize_webcam()
        else:
            self._initialize_cameras()

    def _initialize_webcam(self):
        """Initialize default webcam."""
        try:
            self.cameras['webcam'] = cv2.VideoCapture(0)
        except Exception as e:
            raise ValueError(f"No available webcam found: {e}") from e

    def _initialize_cameras(self):
        """Initialize IDS, Basler, and Luxonis cameras."""
        try:
            self.cameras['ids'] = Ids(camera_id="ids")
            self.cameras['ids'].initialize()
            publish("IDS camera initialized")

            self.cameras['basler'] = Basler(camera_id="basler")
            self.cameras['basler'].initialize()
            publish("Basler camera initialized")

            self.cameras['luxonis'] = Luxonis(camera_id="luxonis")
            self.cameras['luxonis'].initialize()
            publish("Luxonis camera initialized")

        except Exception as e:
            raise ValueError(f"Cameras initialization failed: {e}") from e

    def configure_stream(self):
        """Loads camera configuration files and applies them."""

        check_file_readable(ids_configfile)
        check_file_readable(basler_configfile)

        try:
            self.cameras['ids'].configure(ids_configfile)
            self.cameras['basler'].configure(basler_configfile)
            self.cameras['luxonis'].configure(None)

        except Exception as e:
            raise ValueError(f"Cameras streaming configuration failed: {e}") from e

    def configure_process(self, target_camera):

        check_file_readable(ids_configfile)
        check_file_readable(basler_configfile)

        try:
            self.cameras[target_camera].configure(
                ids_configfile if target_camera == 'ids'
                else basler_configfile if target_camera == 'basler'
                else None
            )
        except Exception as e:
            raise ValueError(f"Camera {target_camera} process configuration failed: {e}") from e

    def start_acquisition(self, cam_name):
        """Starts acquisition for a given camera."""
        try:
            if not self.testing:
                self.cameras[cam_name].acquisition_start()
        except Exception as e:
            raise ValueError(f"Camera acquisition start failed: {e}") from e

    def capture_frame(self, cam_name):
        """Captures a frame from a specified camera."""
        try:
            with self.lock:
                if self.testing and cam_name == 'webcam':
                    ret, frame = self.cameras['webcam'].read()
                    return frame if ret else None
                elif cam_name in self.cameras:
                    return self.cameras[cam_name].capture_frame()
            return None
        except Exception as e:
            raise ValueError(f"Frame capturing failed: {e}") from e

    def stop_acquisition(self, cam_name):
        """Stops acquisition for a given camera."""
        self.cameras[cam_name].acquisition_stop()


# === STREAMING HANDLER ===

class StreamingHandler:
    """Handles continuous streaming of frames from cameras to MQTT."""

    def __init__(self, camera_manager):
        self.camera_manager = camera_manager
        self.threads = {}
        self.running = threading.Event()
        self.cameras = []

    def start(self):
        """Starts streaming threads for all cameras."""
        self.running.set()
        self.cameras = ['webcam', 'webcam', 'webcam'] if self.camera_manager.testing else ['ids', 'basler', 'luxonis']

        for cam in self.cameras:
            self.threads[cam] = threading.Thread(target=self.stream_camera, args=(cam,))
            self.threads[cam].start()

    def stream_camera(self, cam_name):
        """Captures and publishes frames from a single camera."""
        try:
            self.camera_manager.start_acquisition(cam_name)
            while self.running.is_set():
                next_publish_time = time.time() + 0.5
                frame = self.camera_manager.capture_frame(cam_name)

                if frame is not None:
                    encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])[1].tobytes()
                    if encoded:
                        if cam_name == 'webcam':
                            for topic in ['ids', 'basler', 'luxonis']:
                                mqttc.publish(f"optisort/{topic}/stream", im2json(encoded))
                        else:
                            mqttc.publish(f"optisort/{cam_name}/stream", im2json(encoded))

                time.sleep(max(next_publish_time - time.time(), 0))
        except Exception as e:
            raise ValueError(f"Streaming failed: {e}") from e

    def stop(self):
        """Stops all streaming threads and camera acquisitions."""
        self.running.clear()
        for cam in self.cameras:
            self.camera_manager.stop_acquisition(cam)
        for thread in self.threads.values():
            thread.join()


# === PROCESSING HANDLER ===

class ProcessingHandler(threading.Thread):
    """Handles image acquisition and processing for a specific camera."""

    def __init__(self, camera_manager, target_camera):
        super().__init__()
        self.camera_manager = camera_manager
        self.target_camera = target_camera
        self.running = threading.Event()
        self.running.set()
        self.processor = ImageProcessor()

    def run(self):
        """Continuously captures, processes, and publishes frames."""
        try:
            self.camera_manager.start_acquisition(self.target_camera)
            while self.running.is_set():
                frame = self.camera_manager.capture_frame(self.target_camera)
                if frame is not None:
                    self.processor.calculate_image_quality(frame)
                    encoded, buffer = cv2.imencode('.jpg', frame)
                    if encoded:
                        mqttc.publish(f"optisort/{self.target_camera}/stream", im2json(buffer))
                time.sleep(0.01)
        except Exception as e:
            raise ValueError(f"Processing failed: {e}") from e

    def stop(self):
        """Stops processing and publishes final metrics."""
        publish("Processing ended", self.processor.get_metrics())
        self.running.clear()


# === STATE MACHINE ===

states = ['init', 'idle', 'streaming', 'processing', 'ended']

class StateMachine:
    """Manages system states and transitions (initialize, configure, stream, process, terminate)."""

    def __init__(self):
        self.machine = Machine(model=self, states=states, initial='init')
        self.machine.add_transition('initialize', 'init', 'idle', after=self.idle)
        self.machine.add_transition('set_idle', '*', 'idle', after=self.idle)
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
        """Handles MQTT commands to control system state transitions."""
        print(f"Received message on {msg.topic}: {msg.payload.decode('utf-8')}, state: {self.state}")
        payload = json.loads(msg.payload.decode('utf-8'))
        command = payload.get("command")

        if self.state == 'init':
            if command not in ["webcam", "cameras"]:
                publish("Which optics do you want to use? [webcam, cameras]", None)
                return

            self.testing = (command == "webcam")
            publish(f"Initializing {'webcam' if self.testing else 'cameras'}...", None)

            try:
                self.camera_manager = CameraManager(self.testing)
                self.initialize()
            except Exception as e:
                publish(str(e), None)
                self.terminate()

        else:

            if command == "stream":
                try:
                    self.camera_manager.configure_stream()
                    self.start_stream()
                except Exception as e:
                    publish(str(e), None)
                    self.terminate()

            elif command == "process":
                self.target_camera = payload.get("camera")
                if self.target_camera not in ["ids", "basler", "luxonis"]:
                    publish("Specify which camera to process [ids, basler, luxonis]", None)
                    return
                try:
                    self.camera_manager.configure_process(self.target_camera)
                    self.start_process()
                except Exception as e:
                    publish(str(e), None)
                    self.terminate()

            elif command == "stop":
                if self.state == 'streaming' and self.streaming_handler:
                    self.streaming_handler.stop()
                elif self.state == 'processing' and self.processing_handler:
                    self.processing_handler.stop()
                self.set_idle()

            elif command == "exit":
                self.terminate()

            else:
                publish("Command not recognized", None)

    def idle(self):

        if self.streaming_handler is not None:
            self.streaming_handler.stop()
            self.streaming_handler = None

        if self.processing_handler is not None:
            self.processing_handler.stop()
            self.processing_handler = None

        self.target_camera = None
        publish(f"{'Webcam' if self.testing else 'Cameras'} initialized! Send functioning mode {'[stream]' if self.testing else '[stream, process]'}", 1)


    def stream(self):
        try:
            self.streaming_handler = StreamingHandler(self.camera_manager)
            self.streaming_handler.start()
            publish("Stream started!", 2)
        except Exception as e:
            publish(f"{e}", None)  # publish error message over mqtt

    def process(self):
        try:
            if self.testing:
                publish("Cannot use processing mode while testing", None)
            else:
                self.processing_handler = ProcessingHandler(self.camera_manager, self.target_camera)
                self.processing_handler.start()
                publish("Process started!", 3)
        except Exception as e:
            publish(f"{e}", None)  # publish error message over mqtt

    def exit_script(self):
        try:
            if self.streaming_handler:
                self.streaming_handler.stop()

            if self.processing_handler:
                self.processing_handler.stop()

            self.camera_manager.release()
        except Exception as e:
            print(f"Error while terminating: {e}")
        self.running = False


# === MAIN LOOP ===

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
