"""
DESCRIPTION:
This file is provided with the specific functionalities to interact with the UI-5240CP-M-G camera by IDS
"""

from .base_camera import BaseCamera
from ids_peak import ids_peak
import numpy as np
from ids_peak_ipl import ids_peak_ipl


class Ids(BaseCamera):
    def __init__(self, camera_id):
        super().__init__(camera_id)

        ids_peak.Library.Initialize()

        self.device_manager = ids_peak.DeviceManager.Instance()  # Peak library device manager
        self.device = None  # Camera device object
        self.node_map = None
        self.data_stream = None  # Data stream for image capture
        self.buffer_queue = []  # Queue for managing buffers


    def initialize(self):

        try:
            self.device_manager = ids_peak.DeviceManager.Instance()  # Create instance of the device manager
            self.device_manager.Update()  # Update the device manager

            if self.device_manager.Devices().empty():
                raise RuntimeError("No IDS camera found")

            device_count = self.device_manager.Devices().size()  # open the first openable device
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


    def configure(self, config_path):
        try:
            # Determine the current UEyeParametersetPath (str)
            # value = self.node_map.FindNode("UEyeParametersetPath").Value()

            # Set UEyeParametersetPath to "C:/settings/MyCamera" (str)
            self.node_map.FindNode("UEyeParametersetPath").SetValue(str(config_path))

            # Execute UEyeParametersetLoad
            self.node_map.FindNode("UEyeParametersetLoad").Execute()

            # Check if the command has finished before you continue (optional)
            self.node_map.FindNode("UEyeParametersetLoad").WaitUntilDone()

            print(f"IDS camera - configured successfully!")
        except Exception as e:
            raise RuntimeError(f"Failed to configure IDS camera: {e}")


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