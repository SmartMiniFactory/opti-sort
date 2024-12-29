"""
DESCRIPTION:
This file is provided with the specific functionalities to interact with the UI-5240CP-M-G camera by IDS
"""

from base_camera import BaseCamera
from peak.uds import UDSDeviceManager
from peak.uds import Buffer, ImageFormatConverter
import numpy as np


class Ids(BaseCamera):
    def __init__(self, camera_id, config):
        """
        Initialize the IDS Camera.
        :param camera_id: Unique ID for the camera.
        :param config: Dictionary containing camera-specific configurations.
        """
        super().__init__(camera_id, config)
        self.device_manager = UDSDeviceManager()  # Peak library device manager
        self.device = None  # Camera device object
        self.data_stream = None  # Data stream for image capture
        self.converter = ImageFormatConverter()  # For image conversion (e.g., to RGB)
        self.buffer_queue = []  # Queue for managing buffers

    def initialize(self):
        """
        Discover and open the IDS camera using the Peak library.
        """
        try:
            devices = self.device_manager.devices
            if not devices:
                raise RuntimeError("No IDS devices found!")

            # Select the first available device
            self.device = devices[0]
            self.device.open()

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
            # Apply configuration parameters from `self.config`
            if "exposure" in self.config:
                self.device.node_map.ExposureTime.value = self.config["exposure"]
            if "gain" in self.config:
                self.device.node_map.Gain.value = self.config["gain"]
            if "resolution" in self.config:
                width, height = self.config["resolution"]
                self.device.node_map.Width.value = width
                self.device.node_map.Height.value = height
            if "pixel_format" in self.config:
                self.device.node_map.PixelFormat.value = self.config["pixel_format"]

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