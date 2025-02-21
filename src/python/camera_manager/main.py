

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


    #TODO: change topics from static to dynamic (parsing XML)

    if testing:
        cameras = [webcam, webcam, webcam]
        topics = [
            "optisort/ids/stream",
            "optisort/basler/stream",
            "optisort/luxonis/stream",
        ]
    else:
        cameras = [basler_camera, luxonis_camera, ids_camera]
        topics = [
            "optisort/basler/stream",
            "optisort/luxonis/stream",
            "optisort/ids/stream",
        ]

    # Create and run tasks for each topic independently
    tasks = [
        capture_and_publish(camera, topic, mqtt_client, idx) for idx, (camera, topic) in enumerate(zip(cameras, topics))
    ]

    # Gather all tasks
    try:
        await asyncio.gather(*tasks)
    except asyncio.CancelledError:
        print("Tasks were cancelled.")
    except Exception as e:
        print(f"Error occurred: {e}")


async def capture_and_publish(camera, topic, client, idx):
    global testing, encoding, encode_param

    try:
        while True:

            # Calculate the next publication time (synchronized to a 1-second clock)
            next_publish_time = time.time() + 0.5

            try:

                if testing:
                    ret, frame = camera.read()
                    if not ret:
                        print(f"Failed to capture frame from camera for topic {topic}")
                        continue  # Skip this camera and continue with the next loop iteration

                else:
                    # TODO: change topics to dynamic

                    if topic == 'optisort/luxonis/stream':
                        left, right = camera.capture_frame()
                        frame = None
                        if left is not None:
                            frame = left.getCvFrame()
                        elif right is not None:
                            frame = right.getCvFrame()

                    else:
                        frame = camera.capture_frame()

                # Show frame (optional)
                # cv2.imshow(f"Camera frame - {topic}", frame)

                # Encode the frame
                if frame is not None:
                    encoded_frame = cv2.imencode("." + encoding, frame, encode_param)[1].tobytes()

                    # Publish frame to MQTT topic
                    await client.publish(topic, im2json(encoded_frame))

            except Exception as e:
                print(f"Error capturing frame from camera for topic {topic}: {e}")
                continue  # Skip the current iteration and continue with the next

            # Quit if 'q' is pressed
            if keyboard.is_pressed("q"):
                print("Quit detected. Exiting...")
                break

            # Calculate sleep duration to sync to the clock
            now = time.time()
            sleep_duration = max(next_publish_time - now, 0)  # Avoid negative sleep
            await asyncio.sleep(sleep_duration)

    finally:
        # Clean up resources
        cv2.destroyAllWindows()
        print(f"Stopped capture for topic {topic}")


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
        ids_camera.configure()
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
        # luxonis_camera.configure()
        luxonis_camera.acquisition_start()

    # Instance image processor class
    print("Initializing image processor...")
    processor = ImageProcessor()

    print("Starting loop...")
    asyncio.run(main())
