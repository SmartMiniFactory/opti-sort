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
        self.qLeft = None # Left camera monovision streaming
        self.qRight = None # right camera monovision streaming

    def initialize(self):
        """
        Set up the DepthAI pipeline and initialize the Luxonis device.
        """
        try:
            # Define sources and outputs
            monoLeft = self.pipeline.create(dai.node.MonoCamera)
            monoRight = self.pipeline.create(dai.node.MonoCamera)
            xoutLeft = self.pipeline.create(dai.node.XLinkOut)
            xoutRight = self.pipeline.create(dai.node.XLinkOut)

            xoutLeft.setStreamName('left')
            xoutRight.setStreamName('right')

            # Properties
            monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
            monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
            monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

            # Linking
            monoRight.out.link(xoutRight.input)
            monoLeft.out.link(xoutLeft.input)

            # Create the device AFTER defining the full pipeline
            self.device = dai.Device(self.pipeline)

            # Output queues will be used to get the grayscale frames from the outputs defined above
            self.qLeft = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
            self.qRight = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)

            print(f"Luxonis camera initialized successfully! Using device: {self.device.getCameraSensorNames()}")

        except Exception as e:
            raise RuntimeError(f"Failed to initialize Luxonis camera: {e}")

    def configure(self):
        """
        Configure the Luxonis camera using settings from the provided config dictionary.
        """
        try:


            print("Luxonis camera - configured successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to configure Luxonis camera: {e}")

    def acquisition_start(self):
        """
        Start streaming images from the Luxonis camera.
        """
        try:

            self.qLeft = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
            self.qRight = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)
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
            inLeft = self.qLeft.tryGet() if self.qLeft else None
            inRight = self.qRight.tryGet() if self.qRight else None
            return inLeft, inRight

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

