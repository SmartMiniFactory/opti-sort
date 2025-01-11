"""
DESCRIPTION
This file represents the final version merging all the tests done on other files
This will be exported to a .exe and run on the machinery, controlled by and interacting with the HMI (C#)
Overall, it merges:
- three camera handling
- calibrations
- mqtt communication
- feature recognition
"""

# -------------------------------------------------------------------------------------------------------------------

import pathlib
import xml.etree.ElementTree as ET

import cv2
import numpy as np

from MQTT.mqtt_publisher import MQTTClient
from cameras.ids import Ids
from cameras.basler import Basler
from cameras.luxonis import Luxonis
from processing.image_processor import ImageProcessor
import json
import time
import datetime
import keyboard
import base64


def im2json(imdata):
    jstr = json.dumps({"image": base64.b64encode(imdata).decode('ascii'), "timestamp": datetime.datetime.now().isoformat()})
    return jstr


# -------------------------------------------------------------------------------------------------------------------

# MQTT setup: acquiring parameters from HMI settings (AppConfig file)

script_dir = pathlib.Path(__file__).parent.resolve()
appconfig_file = (script_dir / "../../OptiSort/HMI/App.config").resolve()

# old position

mqtt_broker = '127.0.0.1'
mqtt_port = 1883
mqtt_client_name = "camera_manager" # name of the client connecting to the broker (each client must have unique name)
topics = []

encode_param_png = [cv2.IMWRITE_PNG_COMPRESSION, 0]

# parsing App.config XML
tree = ET.parse(appconfig_file)
for element in tree.getroot():
    if element.tag == "userSettings":
        for setting in element[0]:

            if setting.attrib['name'] == 'mqtt_broker':
                mqtt_broker = setting[0].text

            if setting.attrib['name'] == 'mqtt_topic_idsStream':
                topics.append(setting[0].text)

            if setting.attrib['name'] == 'mqtt_topic_luxonisStream':
                topics.append(setting[0].text)

            if setting.attrib['name'] == 'mqtt_topic_baslerStream':
                topics.append(setting[0].text)

            if setting.attrib['name'] == 'mqtt_port':
                mqtt_port = int(setting[0].text)

# instance MQTT client
mqtt_client = MQTTClient(mqtt_broker, mqtt_port, mqtt_client_name, topics)
mqtt_client.connect()

ids_configfile = (script_dir / "../../OptiSort/HMI/config/camera_configuration.ini").resolve()
"""basler_configfile = (script_dir / "../../OptiSort/HMI/config/camera_config.pfs").resolve()
luxonis_configfile = (script_dir / "../../OptiSort/HMI/config/camera_configuration.ini").resolve()
"""
ids_config = {
    "exposure": 10000,  # Microseconds
    "gain": 1.5,
    "resolution": (1280, 720),  # Width, Height
    "pixel_format": "RGB8"
}

# Define cameras
ids_camera = Ids(camera_id="ids1", config_path=ids_configfile)
"""basler_camera = Basler(camera_id="basler1", config_path=basler_configfile)
luxonis_camera = Luxonis(camera_id="luxonis1", config_path=luxonis_configfile)

# Initialize cameras
ids_camera.initialize()
basler_camera.initialize()
luxonis_camera.initialize()

# Start streaming
ids_camera.start_streaming()
basler_camera.start_streaming()
luxonis_camera.start_streaming()"""


ids_camera.open_camera()
ids_camera.prepare_acquisition()
ids_camera.set_roi(16, 16, 128, 128)
ids_camera.alloc_and_announce_buffers()
ids_camera.start_acquisition()


# image processor instance
#processor = ImageProcessor()

# -------------------------------------------------------------------------------------------------------------------

stream = cv2.VideoCapture(0)

# Main loop
while True:

    # TODO: consider async techniques / think about synchronization; manual screenshots or videostream??

    # Capture frames from each camera
    frame = ids_camera.capture_frame()

    print(frame)

    start = time.time()  # Start time
    _, frame = stream.read()  # Read frame

    # Encode image to PNG format
    _frame = cv2.imencode('.png', frame, encode_param_png)[1].tobytes()

    time.sleep(1)  # streaming performance depends also on publishing frequency; 1/15 (FPS) = 0.66

    # Publish results
    mqtt_client.publish("optisort/ids/stream", im2json(frame))
    """mqtt_client.publish("optisort/basler/stream", im2json(processed_basler))
    mqtt_client.publish("optisort/luxonis/stream", im2json(processed_luxonis))"""

    cv2.imshow("Stream input", frame)  # Show the frame

    # Press q if you want to end the loop
    if cv2.waitKey(1) & keyboard.is_pressed('q'):
        print("\nqQuitting...")
        break

    end = time.time()  # End time
    t = end - start
    fps = 1 / t
    print("FPS: ", np.round(fps, 0), end="\r")  # Print the FPS\"""
