from python.camera_manager.processing.image_processor import ImageProcessor
import paho.mqtt.client as mqtt
import cv2
import json
import time
import os
import sys
import pathlib
import yaml


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
SELF_DESTRUCT_TIMEOUT = 20  # seconds
self_destruct_timer = None

# MQTT SETUP
broker = '127.0.0.1'
port = 1883
client_name = 'camera_calibration'
MQTT_KEEPALIVE_INTERVAL = 500

processor = ImageProcessor()
last_activity_time = time.time()

result = {}


def on_message(client, userdata, msg):
    global last_activity_time, result
    last_activity_time = time.time()

    columns, rows, size = None, None, None

    payload = json.loads(msg.payload.decode('utf-8'))
    columns = payload.get("columns")
    rows = payload.get("rows")
    size = payload.get("size")

    if columns is None or rows is None or size is None:
        publish("Please input columns, rows and size", None)
        return

    for camera in ["ids", "basler", "luxonis"]:
        filename = f"{camera}_RefPlaneCalibration"
        absolute_path = (temp_folder / f"{filename}.jpg").resolve()

        if absolute_path.exists():
            image = cv2.imread(str(absolute_path))
            try:

                scale_x, scale_y, chessboard_origin_px, chessboard_center_px, vis_img = processor.compute_pixel_mm_scale(
                    img=image, grid_size=(columns, rows), square_size_mm=size
                )

                result[camera] = {
                    "scale_x": scale_x,
                    "scale_y": scale_y,
                    "chessboard_origin": chessboard_origin_px,
                    "chessboard_center": chessboard_center_px
                }

                publish(f"Calibration successful for {camera}!", None)
            except Exception as e:
                publish(f"Calibration failed for {camera} camera: {e}", None)
                continue
        else:
            publish(f"No image found in the TEMP folder for {camera} camera. Procedure aborted.", None)
            return

    last_activity_time = time.time()  # refresh to avoid program closing during file writing
    with open(config_folder / f"ReferenceFrameCalibration.yaml", "w") as f:
        yaml.dump(result, f)
        publish(f"Reference frame calibration data saved to .yaml file", None)
    sys.exit(0)


# === Main logic ===

mqttc = mqtt.Client()  # Initiate MQTT Client
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.connect(broker, port, MQTT_KEEPALIVE_INTERVAL)  # Connect with MQTT Broker
publish("Calibration script started. Send grid dimensions and square size...", None)

last_activity_time = time.time()

while True:
    mqttc.loop(timeout=1.0)  # Process network events (non-blocking, ~1s)
    elapsed = time.time() - last_activity_time
    if elapsed > SELF_DESTRUCT_TIMEOUT:
        publish("Timeout reached. Exiting.", None)
        sys.exit(0)


