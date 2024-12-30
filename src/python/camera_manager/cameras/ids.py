"""
DESCRIPTION:
This file is provided with the specific functionalities to interact with the UI-5240CP-M-G camera by IDS
"""

from base_camera import BaseCamera
from ids_peak import ids_peak
import configparser
import numpy as np


class Ids(BaseCamera):
    def __init__(self, camera_id, config_path):
        """
        Initialize the IDS Camera.
        :param camera_id: Unique ID for the camera.
        :param config_path: path to the configuration file exported by the manufacturer's application.
        """
        super().__init__(camera_id, config_path)
        ids_peak.Library.Initialize()
        self.device_manager = ids_peak.DeviceManager.Instance()  # Peak library device manager
        self.device = None  # Camera device object
        self.data_stream = None  # Data stream for image capture
        self.buffer_queue = []  # Queue for managing buffers

    def initialize(self):
        """
        Discover and open the IDS camera using the Peak library.
        """
        try:
            self.device_manager.Update()
            if not self.device_manager.Devices().empty():
                raise RuntimeError("No IDS devices found!")

            # Select the first available device
            # https://www.1stvision.com/cameras/IDS/IDS-manuals/en/program-open-camera.html

            self.device = self.device_manager.Devices()[0].OpenDevice(ids_peak.DeviceAccessType_Control)

            # Get the default data stream
            self.data_stream = self.device.data_streams[0]
            self.data_stream.start_acquisition()

            print(f"Camera {self.camera_id} initialized successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize IDS camera: {e}")

    def configure(self):
        """
        Configure the IDS camera using settings from the provided config dictionary.
        """
        try:
            # Extract from config file
            config = configparser.ConfigParser()
            config.read(self.config_path, encoding='cp1250')

            # Loading configuration to camera
            # https://www.1stvision.com/cameras/IDS/IDS-manuals/en/operate-camera-reference.html

            # self.device.node_map.ExposureTime.value =
            # self.device.node_map.Gain.value =
            # self.device.node_map.Width.value = config.get('Image size', 'Width')
            # self.device.node_map.Height.value = config.get('Image size', 'Width')
            # self.device.node_map.PixelFormat.value =

            # TODO: maybe is possible to import/export directly some .cset files
            # https://www.1stvision.com/cameras/IDS/IDS-manuals/en/program-save-cset-file.html

            print(f"Camera {self.camera_id} configured successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to configure IDS camera: {e}")

    def start_streaming(self):
        """
        Start streaming images by queuing buffers for acquisition.
        """
        try:
            # Allocate buffers and queue them for acquisition
            for _ in range(5):  # Number of buffers to allocate
                buffer = self.data_stream.allocate_and_announce_buffer()
                self.buffer_queue.append(buffer)
                self.data_stream.queue_buffer(buffer)

            self.data_stream.start_acquisition()
            print(f"Camera {self.camera_id} started streaming.")
        except Exception as e:
            raise RuntimeError(f"Failed to start streaming for IDS camera: {e}")

    def stop_streaming(self):
        """
        Stop the streaming process and release resources.
        """
        try:
            self.data_stream.stop_acquisition()
            self.device.close()
            print(f"Camera {self.camera_id} stopped streaming.")
        except Exception as e:
            raise RuntimeError(f"Failed to stop streaming for IDS camera: {e}")

    def capture_frame(self):
        """
        Capture a single frame from the IDS camera stream.
        :return: Captured frame as a NumPy array (RGB format).
        """
        try:
            buffer = self.data_stream.wait_for_finished_buffer(5000)  # Wait for 5 seconds
            if buffer:
                # Convert buffer to an image
                image = self.converter.convert(buffer)
                frame = np.asarray(image.get_numpy_array())

                # Re-queue the buffer for further use
                self.data_stream.queue_buffer(buffer)
                return frame
            else:
                raise RuntimeError("Failed to capture frame: Timeout waiting for buffer.")
        except Exception as e:
            raise RuntimeError(f"Failed to capture frame from IDS camera: {e}")