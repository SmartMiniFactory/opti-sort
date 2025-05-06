import numpy as np
import paho.mqtt.client as mqtt
import cv2
import json
import time
import os
import sys
import pathlib
import yaml

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from python.camera_manager.processing.image_processor import ImageProcessor

# SCRIPT IDENTIFICATION (get script path and define TEMP, CONFIG folder paths)
script_dir = pathlib.Path(__file__).parent.resolve()
script_id = str(os.getpid())
script_name = pathlib.Path(__file__).name
temp_folder = script_dir / "../../OptiSort/HMI/Temp"
config_folder = script_dir / "../../OptiSort/HMI/Config"


def publish(message, result):
    global script_dir
    if result is None:
        payload = {"script": {"path": (script_dir / script_name).as_posix(), "PID": script_id}, "message": message}
    else:
        payload = {"script": {"path": (script_dir / script_name).as_posix(), "PID": script_id}, "message": message, "result": result}
    mqttc.publish('optisort/reference_calibration/output', str(json.dumps(payload)))


def on_connect(client, userdata, flags, rc):
    client.subscribe("optisort/reference_calibration/input")


# Timeout setup
SELF_DESTRUCT_TIMEOUT = 10  # seconds
self_destruct_timer = None

# MQTT SETUP
broker = '127.0.0.1'
port = 1883
client_name = 'camera_calibration'
MQTT_KEEPALIVE_INTERVAL = 500

processor = ImageProcessor()
last_activity_time = time.time()

result = {}


def numpy_to_native(obj):
    if isinstance(obj, dict):
        return {k: numpy_to_native(v) for k, v in obj.items()}
    elif isinstance(obj, list) or isinstance(obj, tuple):
        return [numpy_to_native(v) for v in obj]
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.float16, np.float32, np.float64)):
        return float(obj)
    elif isinstance(obj, (np.int_, np.int16, np.int32, np.int64)):
        return int(obj)
    else:
        return obj


def on_message(client, userdata, msg):
    global last_activity_time, result
    last_activity_time = time.time()

    columns, rows, size = None, None, None

    payload = json.loads(msg.payload.decode('utf-8'))
    columns = payload.get("columns")
    rows = payload.get("rows")
    size = payload.get("size")
    scara_center_positioning = payload.get("center") # a point (x, y) in scara coordinates
    scara_yaw = payload.get("yaw")


    if columns is None or rows is None or size is None:
        publish("Please input columns, rows and size", None)
        return

    for camera in ["ids", "basler", "luxonis"]:
        filename = f"{camera}_RefPlaneCalibration"
        absolute_path = (temp_folder / f"{filename}.jpg").resolve()

        if absolute_path.exists():
            image = cv2.imread(str(absolute_path))
            try:

                scale_x, scale_y, chessboard_center_px, vis_img = processor.compute_pixel_mm_scale(
                    img=image, grid_size=(columns, rows), square_size_mm=size
                )

                result[camera] = {
                    "scale_x": scale_x,
                    "scale_y": scale_y,
                    "chessboard_center_px": chessboard_center_px,
                    "chessboard_scara": scara_center_positioning,
                    "chessboard_yaw": scara_yaw
                }

            except Exception as e:
                publish(f"Calibration failed for {camera} camera: {e}", None)
                break
        else:
            publish(f"No image found in the TEMP folder for {camera} camera. Procedure aborted.", None)
            break

    last_activity_time = time.time()  # refresh to avoid program closing during file writing

    if len(result) == 3:
        with open(config_folder / f"ReferenceFrameCalibration.yaml", "w") as f:
            yaml.dump(numpy_to_native(result), f)
            publish(f"Reference frame calibration data saved to .yaml file", 1)

    mqttc.loop(timeout=0.1)  # process network events for a short time
    time.sleep(0.2)  # small wait to ensure packets flush
    mqttc.disconnect()  # clean disconnect (flush outgoing messages)
    sys.exit(0)


# === Main logic ===

mqttc = mqtt.Client()  # Initiate MQTT Client
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.connect(broker, port, MQTT_KEEPALIVE_INTERVAL)  # Connect with MQTT Broker
publish("Calibration script started. Send grid dimensions and square size...", None)

last_activity_time = time.time()

while True:
    mqttc.loop(timeout=0.1)
    elapsed = time.time() - last_activity_time
    if elapsed > SELF_DESTRUCT_TIMEOUT:
        publish("Timeout reached. Exiting.", None)
        sys.exit(0)


