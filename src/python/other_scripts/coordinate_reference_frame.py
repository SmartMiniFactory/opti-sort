from camera_manager.processing.image_processor import ImageProcessor
import paho.mqtt.client as mqtt
import cv2
import json
import os
import pathlib


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
    mqttc.publish('optisort/camera_calibration', str(json.dumps(payload)))


# MQTT SETUP
broker = '127.0.0.1'
port = 1883
client_name = 'camera_calibration'
MQTT_KEEPALIVE_INTERVAL = 500

mqttc = mqtt.Client()  # Initiate MQTT Client
mqttc.connect(broker, port, MQTT_KEEPALIVE_INTERVAL)  # Connect with MQTT Broker
publish("Calibration script starting", None)

processor = ImageProcessor()

# CONFIGURATION
grid_size = (5, 7)  # cols, rows inner corners
square_size_mm = 4.5

# Find saved images
for camera in ["ids", "basler", "luxonis"]:
    filename = f"{camera}_RefPlaneCalibration"
    absolutePath = (temp_folder / f"{filename}.jpg").resolve()

    if absolutePath.exists():
        image = cv2.imread(str(absolutePath))
        scale_x, scale_y, chessboard_origin_px, chessboard_center_px, vis_img = processor.compute_pixel_mm_scale(image, grid_size, square_size_mm)

        if vis_img is not None:
            cv2.imshow("Chessboard ", cv2.resize(vis_img, None, fx=0.5, fy=0.5))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    else:
        print(f"No image found for {camera} camera")

