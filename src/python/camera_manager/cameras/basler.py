"""
DESCRIPTION:
This file is provided with the specific functionalities to interact with the cA1300-32gm camera by BASLER
"""
import os

from python.camera_manager.cameras.base_camera import BaseCamera
from pypylon import pylon


class Basler(BaseCamera):
    def __init__(self, camera_id):
        """
        Initialize the Basler Camera.
        :param camera_id: Unique ID for the camera.
        :param config_path: path to the configuration file exported by the manufacturer's application.
        """
        super().__init__(camera_id)
        self.camera = None  # Basler camera object
        self.converter = pylon.ImageFormatConverter()  # For converting frames to OpenCV-compatible format
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def initialize(self):
        """
        Initialize the Basler camera.
        """
        try:
            # Find and initialize the camera
            tl_factory = pylon.TlFactory.GetInstance()
            devices = tl_factory.EnumerateDevices()
            if not devices:
                raise RuntimeError("No Basler cameras found.")


            self.camera = pylon.InstantCamera(tl_factory.CreateFirstDevice()) # Create an InstantCamera object for the first camera found
            self.camera.Open()  # Open the camera for configuration and streaming

            print(f"Basler camera initialized successfully! Using device: {self.camera.GetDeviceInfo().GetModelName()}")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize Basler camera: {e}")

    def configure(self, config_path):
        """
        Load a PFS file into the Basler camera configuration.
        """
        try:

            if not config_path.is_file():
                raise FileNotFoundError(f"The file {config_path} does not exist.")
            if not os.access(config_path, os.R_OK):  # os.access can still be used for checking readability
                raise PermissionError(f"The file {config_path} is not readable.")

            pylon.FeaturePersistence.Load(str(config_path), self.camera.GetNodeMap(), True)
            print(f"Basler camera - configured successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to configure Basler camera: {e}")

    def acquisition_start(self):
        """
        Start streaming images from the Basler camera.
        """
        try:
            if not self.camera.IsOpen():
                raise RuntimeError("Camera is not initialized. Call `initialize()` first.")
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            print(f"Basler camera - acquisition started successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to start streaming for Basler camera: {e}")

    def acquisition_stop(self):
        """
        Stop streaming images and close the camera.
        """
        try:
            if self.camera and self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            if self.camera and self.camera.IsOpen():
                self.camera.Close()
            print(f"Basler camera - acquisition stopped successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to stop streaming for Basler camera: {e}")

    def capture_frame(self):
        """
        Capture a single frame from the Basler camera stream.
        :return: Captured frame as a NumPy array (BGR format).
        """
        try:
            if not self.camera.IsGrabbing():
                raise RuntimeError("Camera is not streaming. Call `start_streaming()` first.")

            grab_result = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)  # Timeout = 5000ms

            if grab_result.GrabSucceeded():
                # Convert the grabbed buffer to a NumPy array
                frame = self.converter.Convert(grab_result).GetArray()
                grab_result.Release()  # Release the buffer
                return frame
            else:
                raise RuntimeError(f"Frame capture failed: {grab_result.ErrorCode} - {grab_result.ErrorDescription}")
        except Exception as e:
            raise RuntimeError(f"Failed to capture frame from Basler camera: {e}")
