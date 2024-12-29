"""
DESCRIPTION:
This file is provided with the specific functionalities to interact with the cA1300-32gm camera by BASLER
"""

from base_camera import BaseCamera
from pypylon import pylon


class Basler(BaseCamera):
    def __init__(self, camera_id, config):
        """
        Initialize the Basler Camera.
        :param camera_id: Unique ID for the camera.
        :param config: Dictionary containing camera-specific configurations.
        """
        super().__init__(camera_id, config)
        self.camera = None  # Basler camera object
        self.converter = pylon.ImageFormatConverter()  # For converting frames to OpenCV-compatible format
        self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    def initialize(self):
        """
        Initialize the Basler camera.
        """
        try:
            # Create an InstantCamera object for the first camera found
            self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
            self.camera.Open()  # Open the camera for configuration and streaming

            print(f"Camera {self.camera_id} initialized successfully! Using device: {self.camera.GetDeviceInfo().GetModelName()}")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize Basler camera: {e}")

    def configure(self):
        """
        Configure the Basler camera using settings from the provided config dictionary.
        """
        try:
            # Apply configuration parameters from `self.config`
            if "exposure" in self.config:
                self.camera.ExposureTime.SetValue(self.config["exposure"])
            if "gain" in self.config:
                self.camera.Gain.SetValue(self.config["gain"])
            if "pixel_format" in self.config:
                self.camera.PixelFormat.SetValue(self.config["pixel_format"])
            if "frame_rate" in self.config:
                self.camera.AcquisitionFrameRateEnable.SetValue(True)
                self.camera.AcquisitionFrameRate.SetValue(self.config["frame_rate"])

            print(f"Camera {self.camera_id} configured successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to configure Basler camera: {e}")

    def start_streaming(self):
        """
        Start streaming images from the Basler camera.
        """
        try:
            if not self.camera.IsOpen():
                raise RuntimeError("Camera is not initialized. Call `initialize()` first.")
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            print(f"Camera {self.camera_id} started streaming.")
        except Exception as e:
            raise RuntimeError(f"Failed to start streaming for Basler camera: {e}")

    def stop_streaming(self):
        """
        Stop streaming images and close the camera.
        """
        try:
            if self.camera and self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            if self.camera and self.camera.IsOpen():
                self.camera.Close()
            print(f"Camera {self.camera_id} stopped streaming.")
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
