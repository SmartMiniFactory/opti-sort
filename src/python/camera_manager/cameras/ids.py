"""
DESCRIPTION:
This file is provided with the specific functionalities to interact with the UI-5240CP-M-G camera by IDS
"""

from python.camera_manager.cameras.base_camera import BaseCamera
from ids_peak import ids_peak
import numpy as np
from ids_peak_ipl import ids_peak_ipl


class Ids(BaseCamera):
    def __init__(self, camera_id, config_path):
        """
        Initialize the IDS Camera.
        :param camera_id: Unique ID for the camera.
        :param config_path: Path to the configuration file exported by the manufacturer's application.
        """
        super().__init__(camera_id, config_path)

        ids_peak.Library.Initialize()

        self.device_manager = ids_peak.DeviceManager.Instance()  # Peak library device manager
        self.device = None  # Camera device object
        self.node_map = None
        self.data_stream = None  # Data stream for image capture
        self.buffer_queue = []  # Queue for managing buffers


    def initialize(self):
        """
        Discover and open the IDS camera using the Peak library.
        """
        try:
            # Create instance of the device manager
            self.device_manager = ids_peak.DeviceManager.Instance()

            # Update the device manager
            self.device_manager.Update()

            # Return if no device was found
            if self.device_manager.Devices().empty():
                raise RuntimeError("No IDS camera found")

            # open the first openable device in the device manager's device list
            device_count = self.device_manager.Devices().size()
            for i in range(device_count):
                if self.device_manager.Devices()[i].IsOpenable():
                    self.device = self.device_manager.Devices()[i].OpenDevice(ids_peak.DeviceAccessType_Control)

                    # Get NodeMap of the RemoteDevice for all accesses to the GenICam NodeMap tree
                    self.node_map = self.device.RemoteDevice().NodeMaps()[0]
                    break

            # prepare acquisition
            self.data_stream = self.device.DataStreams()
            if self.data_stream.empty():
                raise RuntimeError("No Data Stream available")


            self.data_stream = self.device.DataStreams()[0].OpenDataStream()
            print(f"IDS camera initialized successfully! Using device: {self.device.DisplayName()}")

            return True

        except Exception as e:
            raise RuntimeError(f"Failed to initialize IDS camera: {e}")


    def configure(self):
        """
        Configure the IDS camera using settings from the provided config dictionary.
        """
        try:
            # Extract from config file
            # config = configparser.ConfigParser()
            # config.read(self.config_path, encoding='cp1250')

            # Loading configuration to camera
            # https://www.1stvision.com/cameras/IDS/IDS-manuals/en/operate-camera-reference.html

            # self.device.node_map.ExposureTime.value =
            # self.device.node_map.Gain.value =
            # self.device.node_map.Width.value = config.get('Image size', 'Width')
            # self.device.node_map.Height.value = config.get('Image size', 'Width')
            # self.device.node_map.PixelFormat.value =

            """node_map = self.device.RemoteDevice().NodeMaps()[0]
            node_map.FindNode("Width").SetValue(1280)  # Set width
            node_map.FindNode("Height").SetValue(720)  # Set height
            node_map.FindNode("PixelFormat").SetValue("Mono8")  # Example: 8-bit monochrome

            node_map.FindNode("AcquisitionMode").SetValue("Continuous")"""

            # Set camera to continuous acquisition mode
            self.node_map.FindNode("AcquisitionMode").SetCurrentEntry("Continuous")

            # Disable trigger mode

            self.node_map.FindNode("TriggerMode").SetCurrentEntry("Off")
            # self.node_map.FindNode("TriggerSelector").SetCurrentEntry("AcquisitionStart")
            self.node_map.FindNode("AcquisitionFrameRate").SetValue(17.2)


            # Enable auto exposure
            # self.node_map.FindNode("ExposureAuto").SetCurrentEntry("Off")

            # Enable auto gain
            # self.node_map.FindNode("GainAuto").SetCurrentEntry("Off")

            # Optionally, set the frame rate if needed
            # self.node_map.FindNode("AcquisitionFrameRate").SetValue(30)  # Set to desired frame rate (e.g., 30 FPS)

            # TODO: maybe is possible to import/export directly some .cset files
            # https://www.1stvision.com/cameras/IDS/IDS-manuals/en/program-save-cset-file.html

            # List all available nodes in the NodeMap
            # for node in self.node_map.Nodes():
                # print(node.Name())

            print(f"IDS camera - configured successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to configure IDS camera: {e}")


    def set_roi(self, offset_x, offset_y, width, height):
        try:
            # Get the minimum ROI and set it. After that there are no size restrictions anymore
            x_min = self.node_map.FindNode("OffsetX").Minimum()
            y_min = self.node_map.FindNode("OffsetY").Minimum()
            w_min = self.node_map.FindNode("Width").Minimum()
            h_min = self.node_map.FindNode("Height").Minimum()

            self.node_map.FindNode("OffsetX").SetValue(x_min)
            self.node_map.FindNode("OffsetY").SetValue(y_min)
            self.node_map.FindNode("Width").SetValue(w_min)
            self.node_map.FindNode("Width").SetValue(w_min)
            self.node_map.FindNode("Height").SetValue(h_min)

            # Get the maximum ROI values
            x_max = self.node_map.FindNode("OffsetX").Maximum()
            y_max = self.node_map.FindNode("OffsetY").Maximum()
            w_max = self.node_map.FindNode("Width").Maximum()
            h_max = self.node_map.FindNode("Height").Maximum()

            if (offset_x < x_min) or (offset_y < y_min) or (offset_x > x_max) or (offset_y > y_max):
                raise RuntimeError("Wrong ROI setup: parameters outside the maximum bounds")
            elif (width < w_min) or (height < h_min) or ((offset_x + width) > w_max) or ((offset_y + height) > h_max):
                raise RuntimeError("Wrong ROI setup: exceeding width or height")

            else:
                # Now, set final AOI
                self.node_map.FindNode("OffsetX").SetValue(offset_x)
                self.node_map.FindNode("OffsetY").SetValue(offset_y)
                self.node_map.FindNode("Width").SetValue(width)
                self.node_map.FindNode("Height").SetValue(height)

                print(f"IDS camera - ROI set successfully!")
                return True

        except Exception as e:
            raise RuntimeError(f"Failed to set ROI for IDS camera: {e}")


    def acquisition_start(self):
        """
        Start streaming images by queuing buffers for acquisition.
        """
        try:
            # allocate and announce buffer
            if self.data_stream:
                # Flush queue and prepare all buffers for revoking
                self.data_stream.Flush(ids_peak.DataStreamFlushMode_DiscardAll)

                # Clear all old buffers
                for buffer in self.data_stream.AnnouncedBuffers():
                    self.data_stream.RevokeBuffer(buffer)

                payload_size = self.node_map.FindNode("PayloadSize").Value()

                # Get number of minimum required buffers
                num_buffers_min_required = self.data_stream.NumBuffersAnnouncedMinRequired()

                # Alloc buffers
                for count in range(num_buffers_min_required):
                    buffer = self.data_stream.AllocAndAnnounceBuffer(payload_size)
                    self.data_stream.QueueBuffer(buffer)

                # Start acquisition
                self.data_stream.StartAcquisition(ids_peak.AcquisitionStartMode_Default,
                                                  ids_peak.DataStream.INFINITE_NUMBER)
                self.node_map.FindNode("TLParamsLocked").SetValue(1)
                self.node_map.FindNode("AcquisitionStart").Execute()
                print(f"IDS camera - acquisition started successfully!")
                return True

        except Exception as e:
            raise RuntimeError(f"Failed to start streaming for IDS camera: {e}")


    def acquisition_stop(self):
        """
        Stop the streaming process and release resources.
        """
        try:
            # TODO: unrevised; check documentation
            self.data_stream.stop_acquisition()
            self.device.close()
            print(f"IDS camera - acquisition stopped successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to stop streaming for IDS camera: {e}")


    def capture_frame(self):
        try:

            # https://www.1stvision.com/cameras/IDS/IDS-manuals/en/program-convert-images-generic.html

            # Wait for the buffer to be finished (timeout of 5000ms)
            buffer = self.data_stream.WaitForFinishedBuffer(5000)
            if buffer is None:
                raise RuntimeError("No buffer received from IDS camera (timeout or issue).")

            image = ids_peak_ipl.Image.CreateFromSizeAndBuffer(
                buffer.PixelFormat(),
                buffer.BasePtr(),
                buffer.Size(),
                buffer.Width(),
                buffer.Height()
            )

            image = image.ConvertTo(ids_peak_ipl.PixelFormatName_BGRa8, ids_peak_ipl.ConversionMode_Fast)
            img_array = image.get_numpy()

            if img_array is None:
                raise RuntimeError("Image conversion failed, got None.")

            # Ensure correct format for OpenCV
            img_array = img_array.astype(np.uint8)
            img_array = img_array[:, :, :3]  # Remove alpha channel

            self.data_stream.QueueBuffer(buffer)

            return img_array

        except Exception as e:
            raise RuntimeError(f"Failed to capture frame from IDS camera: {e}")