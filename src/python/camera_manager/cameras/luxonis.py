"""
DESCRIPTION:
This file is provided with the specific functionalities to interact with the OAK-D SR PoE/CAD camera by LUXONIS
"""

from cameras.base_camera import BaseCamera
import depthai as dai


class Luxonis(BaseCamera):
    def __init__(self, camera_id, config_path):
        """
        Initialize the Luxonis Camera.
        :param camera_id: Unique ID for the camera.
        :param config_path: path to the configuration file exported by the manufacturer's application.
        """
        super().__init__(camera_id, config_path)
        self.pipeline = dai.Pipeline()  # DepthAI pipeline for configuring streams
        self.device = None  # Luxonis device object
        self.cam_rgb = None  # RGB camera node
        self.xout_video = None  # Output stream for RGB video
        self.rgb_queue = None  # Queue to read RGB frames

    def initialize(self):
        """
        Set up the DepthAI pipeline and initialize the Luxonis device.
        """
        try:
            # Create an RGB camera node
            self.cam_rgb = self.pipeline.create(dai.node.ColorCamera)
            self.cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
            self.cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

            # Configure video output
            self.xout_video = self.pipeline.create(dai.node.XLinkOut)
            self.xout_video.setStreamName("video")
            self.cam_rgb.video.link(self.xout_video.input)

            # Initialize the device with the pipeline
            self.device = dai.Device(self.pipeline)
            self.rgb_queue = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

            print(f"Luxonis camera initialized successfully! Using device: {dai.DeviceBase.getCameraSensorNames()}")
        except Exception as e:
            raise RuntimeError(f"Failed to initialize Luxonis camera: {e}")

    def configure(self):
        """
        Configure the Luxonis camera using settings from the provided config dictionary.
        """
        try:
            # Apply configuration parameters from `self.config`
            if "fps" in self.config:
                self.cam_rgb.setFps(self.config["fps"])
            if "color_order" in self.config:
                self.cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB if self.config[
                    "color_order"] == "RGB" else dai.ColorCameraProperties.ColorOrder.BGR)
            if "resolution" in self.config:
                width, height = self.config["resolution"]
                self.cam_rgb.setVideoSize(width, height)

            print("Luxonis camera - configured successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to configure Luxonis camera: {e}")

    def acquisition_start(self):
        """
        Start streaming images from the Luxonis camera.
        """
        try:
            if not self.rgb_queue:
                raise RuntimeError("RGB queue is not initialized. Call `initialize()` first.")
            print("Luxonis camera - acquisition started successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to start streaming for Luxonis camera: {e}")

    def acquisition_stop(self):
        """
        Stop the Luxonis camera device.
        """
        try:
            self.device.close()
            print("Luxonis camera - acquisition stopped successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to stop streaming for Luxonis camera: {e}")

    def capture_frame(self):
        """
        Capture a single frame from the Luxonis camera stream.
        :return: Captured frame as a NumPy array.
        """
        try:
            in_video = self.rgb_queue.get()  # Get a frame from the video stream
            frame = in_video.getCvFrame()  # Convert the frame to OpenCV format (NumPy array)
            return frame
        except Exception as e:
            raise RuntimeError(f"Failed to capture frame from Luxonis camera: {e}")