

# -------------------------------------------------------------------------------------------------------------------

import pathlib
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import asyncio
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


def im2json(imdata):
    jstr = json.dumps(
        {"image": base64.b64encode(imdata).decode('ascii'), "timestamp": datetime.datetime.now().isoformat()})
    return jstr


async def main():
    global testing, webcam, ids_camera, basler_camera, luxonis_camera, mqtt_client

    if testing:
        tasks = [
            capture_and_publish(webcam, "optisort/ids/stream", mqtt_client),
            capture_and_publish(webcam, "optisort/basler/stream", mqtt_client),
            capture_and_publish(webcam, "optisort/luxonis/stream", mqtt_client),
        ]
    else:
        tasks = [
                capture_and_publish(ids_camera, "optisort/ids/stream", mqtt_client),
                capture_and_publish(basler_camera, "optisort/basler/stream", mqtt_client),
                capture_and_publish(luxonis_camera, "optisort/luxonis/stream", mqtt_client),
        ]
    await asyncio.gather(*tasks)


async def capture_and_publish(camera, topic, client):
    global testing, encoding, encode_param

    while True:
        start = time.time()  # Start time

        if testing:
            # Capture frame from the testing camera (webcam)
            ret, frame = camera.read()
            if not ret:
                print("Failed to capture webcam frame.")
                return False
        else:
            # Capture frames from the real cameras
            frame = camera.capture_frame()

        # show frame
        cv2.imshow("Webcam frame", frame)

        # Encode the frame
        encoded_frame = cv2.imencode("." + encoding, frame, encode_param)[1].tobytes()

        # Publish frame to MQTT topics
        await client.publish(topic, im2json(encoded_frame))

        # Handle frame display and quitting
        if cv2.waitKey(1) & keyboard.is_pressed('q'):
            print("\nQuitting...")
            return False

        # Calculate FPS and sleep for the remaining time
        end = time.time()
        elapsed = end - start
        fps = 1 / elapsed
        print("FPS: ", np.round(fps, 0), end="\r")

        if await asyncio.sleep(0.2 - elapsed):  # Adjust interval based on elapsed time
            return True

# -------------------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------------------
# -------------------------------------------------------------------------------------------------------------

if __name__ == "__main__":

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

    # define encoding parameters
    if encoding == "jpg":
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    else:
        encode_param = [cv2.IMWRITE_PNG_COMPRESSION, 0]

    # Define important directories or files
    script_dir = pathlib.Path(__file__).parent.resolve()
    appconfig_file = (script_dir / "../../OptiSort/HMI/App.config").resolve()

    # Initialize MQTT
    mqtt_client_name = "camera_manager"  # name of the client (unique) connecting to the broker
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

    print("Starting loop...")
    asyncio.run(main())
