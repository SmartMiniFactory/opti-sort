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


# -------------------------------------------------------------------------------------------------------------------


def im2json(imdata):
    jstr = json.dumps({"image": base64.b64encode(imdata).decode('ascii'), "timestamp": datetime.datetime.now().isoformat()})
    return jstr


def parse_appconfig(file_path):

    # set defaults
    broker = '127.0.0.1'
    port = 1883
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


# -------------------------------------------------------------------------------------------------------------------


# acquire testing mode
testing = ''
while testing != 'c' and testing != 'w':
    testing = input("Do you want to use your webcam or real cameras? (w/c): ")

if testing == 'c':
    testing = False
else:
    testing = True

# acquire encoding mode
encoding = ''
while encoding != "jpg" and encoding != "png":
    encoding = input("Do you want to jpg or png as encoding mode? (jpg/png): ")

encode_param_png = [cv2.IMWRITE_PNG_COMPRESSION, 0]
encode_param_jpg = [int(cv2.IMWRITE_JPEG_QUALITY), 90]


# Define important directories or files
script_dir = pathlib.Path(__file__).parent.resolve()
appconfig_file = (script_dir / "../../OptiSort/HMI/App.config").resolve()

# Initialize MQTT
mqtt_client_name = "camera_manager" # name of the client (unique) connecting to the broker
mqtt_broker, mqtt_port, mqtt_stream_topics = parse_appconfig(appconfig_file)
mqtt_client = MQTTClient(mqtt_broker, mqtt_port, mqtt_client_name, mqtt_stream_topics)
mqtt_client.connect()

# locate configuration files
ids_configfile = (script_dir / "../../OptiSort/HMI/Config/camera_configuration.ini").resolve()
basler_configfile = (script_dir / "../../OptiSort/HMI/Config/camera_config.pfs").resolve()
luxonis_configfile = (script_dir / "../../OptiSort/HMI/Config/camera_configuration.ini").resolve()

"""ids_config = {
    "exposure": 10000,  # Microseconds
    "gain": 1.5,
    "resolution": (1280, 720),  # Width, Height
    "pixel_format": "RGB8"
}"""


if testing:
    print("Opening webcam...")
    webcam = cv2.VideoCapture(0)

else:
    # Instance IDS camera
    print("Initializing IDS camera...")
    ids_camera = Ids(camera_id="ids", config_path=ids_configfile)
    ids_camera.initialize()
    # ids_camera.configure()
    ids_camera.set_roi(0, 0, 1280, 1024)
    ids_camera.acquisition_start()

    # Instance Basler camera
    print("Initializing Basler camera...")
    basler_camera = Basler(camera_id="basler", config_path=basler_configfile)
    basler_camera.initialize()
    basler_camera.acquisition_start()

    # Instance Luxonis camera
    print("Initializing Luxonis camera...")
    luxonis_camera = Luxonis(camera_id="luxonis", config_path=luxonis_configfile)
    luxonis_camera.initialize()
    luxonis_camera.acquisition_start()


# Instance image processor class
print("Initializing image processor...")
processor = ImageProcessor()

# -------------------------------------------------------------------------------------------------------------------


print("Starting loop...")

# Main loop
while True:

    start = time.time()  # Start time

    # TODO: consider async techniques / think about synchronization; manual screenshots or videostream??

    # Capture frames from each camera
    if testing:
        _, webcam_frame = webcam.read()
        cv2.imshow("Webcam frame", webcam_frame)

        # encode to PNG or JPG and then convert to JSON to publish mqtt message
        """expression_if_true if condition else expression_if_false"""
        encoded_webcam_frame = cv2.imencode('.png', webcam_frame, encode_param_png if encoding == "png" else encode_param_jpg)[1].tobytes()

        # comment out below to simulate camera malfunctioning
        mqtt_client.publish("optisort/ids/stream", im2json(encoded_webcam_frame))
        mqtt_client.publish("optisort/basler/stream", im2json(encoded_webcam_frame))
        mqtt_client.publish("optisort/luxonis/stream", im2json(encoded_webcam_frame))


    else:
        ids_frame = ids_camera.capture_frame()
        basler_frame = basler_camera.capture_frame()
        luxonis_frame = luxonis_camera.capture_frame()

        cv2.imshow("IDS frame", ids_frame)
        cv2.imshow("Basler frame", basler_frame)
        cv2.imshow("Luxonis frame", luxonis_frame)

        # encode to PNG or JPG and then convert to JSON to publish mqtt message
        encoded_ids_frame = cv2.imencode('.png', ids_frame, encode_param_png if encoding == "png" else encode_param_jpg)[1].tobytes()
        encoded_basler_frame = cv2.imencode('.png', basler_frame, encode_param_png if encoding == "png" else encode_param_jpg)[1].tobytes()
        encoded_luxonis_frame = cv2.imencode('.png', luxonis_frame, encode_param_png if encoding == "png" else encode_param_jpg)[1].tobytes()

        mqtt_client.publish("optisort/ids/stream", im2json(encoded_ids_frame))
        mqtt_client.publish("optisort/basler/stream", im2json(encoded_basler_frame))
        mqtt_client.publish("optisort/luxonis/stream", im2json(encoded_luxonis_frame))


    # Press q if you want to end the loop
    if cv2.waitKey(1) & keyboard.is_pressed('q'):
        print("\nqQuitting...")
        break

    end = time.time()  # End time
    t = end - start
    fps = 1 / t
    print("FPS: ", np.round(fps, 0), end="\r")  # Print the FPS\"""

    time.sleep(0.2)  # streaming performance depends also on publishing frequency; 1/15 (FPS) = 0.66