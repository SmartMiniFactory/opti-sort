# === IMPORTS ===
import os
import threading
import time
import datetime
import pathlib
import base64
import json
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
# os.add_dll_directory(r"C:\Program Files\Basler\pylon 8\Runtime\Win32")

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

    def __init__(self, testing=False, target_camera=None):
        self.testing = testing
        self.cameras = {}
        self.lock = threading.Lock()
        self.target_camera = target_camera

        if testing:
            self._initialize_webcam()
        else:
            self._initialize_cameras(target_camera)

    def _initialize_webcam(self):
        """Initialize default webcam."""
        try:
            self.cameras['webcam'] = cv2.VideoCapture(0)
        except Exception as e:
            raise ValueError(f"No available webcam found: {e}") from e

    def _initialize_cameras(self, target_camera):
        try:
            for camera in target_camera:
                self.cameras[camera] = (
                    Ids(camera_id=camera) if camera == "ids" else
                    Basler(camera_id=camera) if camera == "basler" else
                    Luxonis(camera_id=camera) if camera == "luxonis" else None
                )
                self.cameras[camera].initialize()
                publish(f"{camera} camera initialized")
                mqttc.loop(timeout=0.1)  # force for a short time the main thread to publish mqtt message immediately

            if len(target_camera) > 1:
                self._configure_stream()
            else:
                self._configure_process()

        except Exception as e:
            raise ValueError(f"Cameras initialization failed: {e}") from e

    def _configure_stream(self):
        """Loads camera configuration for streaming performance."""
        check_file_readable(ids_configfile)
        check_file_readable(basler_configfile)
        try:
            self.cameras['ids'].configure(ids_configfile)
            self.cameras['basler'].configure(basler_configfile)
            self.cameras['luxonis'].configure(None)
        except Exception as e:
            raise ValueError(f"Cameras streaming configuration failed: {e}") from e

    def _configure_process(self):
        """Load camera configuration for processing performance."""
        check_file_readable(ids_configfile)
        check_file_readable(basler_configfile)
        camera = self.target_camera[0]
        try:
            self.cameras[camera].configure(
                ids_configfile if camera == 'ids'
                else basler_configfile if camera == 'basler'
                else None
            )
        except Exception as e:
            raise ValueError(f"Camera {camera} process configuration failed: {e}") from e

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
        with self.lock:
            try:
                self.cameras[cam_name].acquisition_stop()
            except Exception as e:
                publish(f"[WARN] Failed to cleanly shutdown {cam_name}: {e}", None)


# === STREAMING HANDLER ===

class StreamingHandler:

    def __init__(self, camera_manager):
        self.camera_manager = camera_manager
        self.threads = {}
        self.running = threading.Event()
        self.cameras = []

    def run(self):
        """Starts streaming threads for all cameras."""
        self.running.set()
        self.cameras = ['webcam', 'webcam', 'webcam'] if self.camera_manager.testing else ['ids', 'basler', 'luxonis']

        for cam in self.cameras:
            self.threads[cam] = threading.Thread(target=self._stream_camera, args=(cam,))
            self.threads[cam].start()

    def _stream_camera(self, cam_name):
        """Captures and publishes frames from a single camera."""
        try:
            self.camera_manager.start_acquisition(cam_name)
            while self.running.is_set():
                next_publish_time = time.time() + 0.5

                try:
                    frame = self.camera_manager.capture_frame(cam_name)
                except Exception as e:
                    # quitting if camera results closed
                    print(f"[{cam_name}] Capture failed (likely stop): {e}")
                    break

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

class ProcessingHandler:

    def __init__(self, camera_manager, target_camera):
        self.camera_manager = camera_manager
        self.thread = None
        self.running = threading.Event()
        self.target_camera = target_camera[0]
        self.running = threading.Event()

    def run(self):
        """Continuously captures, processes, and publishes frames."""
        self.running.set()
        self.thread = threading.Thread(target=self._process_camera)
        self.thread.start()

    def _process_camera(self):
        try:

            # Robot-provided chessboard center (where robot places center of checkerboard)
            scara_chessboard_center_mm = (432.924, 224.126)  # Example mm, replace with your robot data
            scara_chessboard_yaw_deg = 0

            grid_size = (5, 7)  # cols, rows inner corners
            square_size_mm = 4.5

            self.camera_manager.start_acquisition(self.target_camera)
            proc = ImageProcessor()

            while self.running.is_set():
                # next_publish_time = time.time() + 0.1

                try:
                    frame = self.camera_manager.capture_frame(self.target_camera)
                except Exception as e:
                    # quitting if camera results closed
                    print(f"[{self.target_camera}] Capture failed (likely stop): {e}")
                    break

                if frame is None:
                    continue

                thresh, labeled_image, detected_objects = proc.detect_shapes_and_classify(frame)

                if labeled_image is not None:
                    encoded = cv2.imencode(".jpg", labeled_image, [int(cv2.IMWRITE_JPEG_QUALITY), 90])[
                        1].tobytes()
                    if encoded:
                        mqttc.publish(f"optisort/{self.target_camera}/stream", im2json(encoded))

                if len(detected_objects) > 0:
                    for object in detected_objects:
                        stable_position = proc.stabilize_detection(object)  # Stabilizza la posizione del componente

                        if stable_position is not None:
                            component, stable_x, stable_y, stable_a = stable_position

                            scale_x, scale_y, chessboard_origin_px, chessboard_center_px, vis_img = proc.compute_pixel_mm_scale(
                                frame, grid_size, square_size_mm
                            )

                            detected_pixel = (stable_x, stable_y)

                            X_scara, Y_scara = proc.pixel_to_scara(
                                detected_pixel,
                                chessboard_center_px,
                                scara_chessboard_center_mm,
                                scale_x,
                                scale_y,
                                scara_chessboard_yaw_deg
                            )

                            # ---- ISTERESI ----
                            if proc.should_send_mqtt(component, (X_scara, Y_scara)):

                                payload = {
                                    "script": {
                                        "path": (script_dir / script_name).as_posix(),
                                        "PID": script_id
                                    },
                                    "message": {
                                        "type": component,
                                        "x": X_scara,
                                        "y": Y_scara,
                                        "z": 0.0,
                                        "rx": 0.0,
                                        "ry": 0.0,
                                        "rz": 999.9
                                    }
                                }

                                mqttc.publish('optisort/scara/target', str(json.dumps(payload)), qos=0)

                # time.sleep(max(next_publish_time - time.time(), 0))
        except Exception as e:
            raise ValueError(f"Processing failed: {e}") from e

    def stop(self):
        """Stops processing and publishes final metrics."""
        self.running.clear()
        self.camera_manager.stop_acquisition(self.target_camera)
        self.thread.join()


# === STATE MACHINE ===

states = ['init', 'idle', 'streaming', 'processing',  'ended']

class StateMachine:
    """Manages system states and transitions (initialize, configure, stream, process, terminate)."""

    def __init__(self):
        self.machine = Machine(model=self, states=states, initial='init')
        self.machine.add_transition('initialize', '*', 'idle', after=self.idle)
        self.machine.add_transition('start_stream', 'idle', 'streaming', after=self.stream)
        self.machine.add_transition('start_process', 'idle', 'processing', after=self.process)
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
            mqttc.loop(timeout=0.1)  # force for a short time the main thread to publish mqtt message immediately
            self.initialize()

        else:
            if command == "stream":
                self.start_stream()

            elif command == "process":
                cam = payload.get("camera")
                if cam not in ["ids", "basler", "luxonis"]:
                    publish("Specify which camera to process [ids, basler, luxonis]", None)
                else:
                    self.target_camera = [cam]
                    self.start_process()

            elif command == "stop":
                self.initialize()

            elif command == "exit":
                self.terminate()

            else:
                publish("Command not valid", None)

    def idle(self):

        if self.streaming_handler is not None:
            self.streaming_handler.stop()
            self.streaming_handler = None

        if self.processing_handler is not None:
            self.processing_handler.stop()
            self.processing_handler = None

        self.target_camera = None
        self.camera_manager = None

        publish(f"State machine in idle! Send functioning mode {'[stream]' if self.testing else '[stream, process]'}",1)
        mqttc.loop(timeout=0.1)  # force for a short time the main thread to publish mqtt message immediately

    def stream(self):
        try:
            self.camera_manager = CameraManager(testing=self.testing, target_camera=["ids", "basler", "luxonis"])  # cameras get ignored if testing is True
            self.streaming_handler = StreamingHandler(self.camera_manager)
            self.streaming_handler.run()
            publish("Stream started!", 2)
        except Exception as e:
            publish(f"Streaming initialization error: {e}", None)  # publish error message over mqtt
            self.terminate()

    def process(self):
        if self.testing:
            publish("Cannot use processing mode while testing", None)
            return
        try:
            self.camera_manager = CameraManager(testing=False, target_camera=self.target_camera)
            self.processing_handler = ProcessingHandler(self.camera_manager, self.target_camera)
            self.processing_handler.run()
            publish("Process started!", 3)
        except Exception as e:
            publish(f"Processing error: {e}", None)  # publish error message over mqtt
            self.terminate()


    def exit_script(self):
        try:
            if self.streaming_handler is not None:
                self.streaming_handler.stop()

            if self.processing_handler is not None:
                self.processing_handler.stop()

            self.camera_manager = None
            publish(f"Terminating program", 4)

        except Exception as e:
            publish(f"Error while terminating: {e}")
        self.running = False


# === MAIN LOOP ===

def main():
    mqttc.on_connect = on_connect  # Register connect callback
    mqttc.connect(broker, port, MQTT_KEEPALIVE_INTERVAL)  # Connect with MQTT Broker
    mqttc.subscribe("optisort/camera_manager/input")  # subscribe to topic for receiving commands
    mqttc.loop_start()  # Start a non-blocking separate thread for mqtt communications

    state_machine = StateMachine()  # activate state machine class
    mqttc.on_message = state_machine.on_message  # attach MQTT messages to state machine class

    while state_machine.running:
        time.sleep(0.05)  # Prevents 100% CPU usage

    # Ensure cleanup
    mqttc.loop_stop()
    mqttc.disconnect()


if __name__ == "__main__":
    main()
