"""
DESCRIPTION:
This file is provided with the specific functionalities to interact with the OAK-D SR PoE/CAD camera by LUXONIS
"""

from python.camera_manager.cameras.base_camera import BaseCamera
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
        self.configQueue = None
        self.video = None

    def initialize(self):
        """
        Set up the DepthAI pipeline and initialize the Luxonis device.
        """
        try:
            # Define sources and outputs
            cam = self.pipeline.create(dai.node.Camera)  # RGB camera
            cam.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            cam.setSize((1280, 800))  # Initial resolution selection

            # ImageManip node for cropping an AOI to 1:1 aspect ratio
            manip = self.pipeline.create(dai.node.ImageManip)
            manip.initialConfig.setCropRect(0.3, 0.1, 0.7, 0.9)
            manip.initialConfig.setResize(800, 800)  # Resize after cropping
            manip.setMaxOutputFrameSize(800 * 800 * 3)  # Assuming max square crop is 800x800
            manip.setKeepAspectRatio(False)
            manip.initialConfig.setFrameType(dai.ImgFrame.Type.GRAY8)  # Convert to greyscale

            # Create node connections
            configIn = self.pipeline.create(dai.node.XLinkIn)
            videoOut = self.pipeline.create(dai.node.XLinkOut)

            # Stream names
            configIn.setStreamName('config')
            videoOut.setStreamName("video")

            # Linking nodes
            configIn.out.link(cam.inputConfig)
            cam.video.link(manip.inputImage)
            manip.out.link(videoOut.input)

            # Create the device AFTER defining the full pipeline
            self.device = dai.Device(self.pipeline)

            # Output queues will be used to get the grayscale frames from the outputs defined above
            self.configQueue = self.device.getInputQueue('config')
            self.video = self.device.getOutputQueue(name="video", maxSize=1, blocking=False)

            print(f"Luxonis camera initialized successfully! Using device: {self.device.getCameraSensorNames()}")

        except Exception as e:
            raise RuntimeError(f"Failed to initialize Luxonis camera: {e}")

    def configure(self):
        """
        Configure the Luxonis camera using settings from the provided config dictionary.
        """
        try:

            print(self.pipeline.getCalibrationData())

            print("Luxonis camera - configured successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to configure Luxonis camera: {e}")

    def acquisition_start(self):
        """
        Start streaming images from the Luxonis camera.
        """
        try:

            self.video = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)
            print("Device connected. Queues initialized.")

            print("Luxonis camera - acquisition started successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to start streaming for Luxonis camera: {e}")


    def capture_frame(self):
        """
        Capture a single frame from the Luxonis camera stream.
        :return: Captured frame as a NumPy array.
        """
        try:
            video = self.video.tryGet() if self.video else None
            return video

        except Exception as e:
            raise RuntimeError(f"Failed to capture frame from Luxonis camera: {e}")

    def acquisition_stop(self):
        """
        Stop the Luxonis camera device.
        """
        try:
            if self.device:
                del self.device
            print("Luxonis camera - device disconnected!")
        except Exception as e:
            raise RuntimeError(f"Failed to stop streaming for Luxonis camera: {e}")

