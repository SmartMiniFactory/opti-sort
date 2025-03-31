from ids_peak import ids_peak
import pathlib
import os
from ids_peak_ipl import ids_peak_ipl
import cv2
import numpy as np
import keyboard

ids_peak.Library.Initialize()

device_manager = ids_peak.DeviceManager.Instance()  # Peak library device manager
device = None  # Camera device object
node_map = None
data_stream = None  # Data stream for image capture
buffer_queue = []  # Queue for managing buffers


device_manager.Update()

            # Return if no device was found
if device_manager.Devices().empty():
    raise RuntimeError("No IDS camera found")


device_count = device_manager.Devices().size()

for i in range(device_count):
    if device_manager.Devices()[i].IsOpenable():
        device = device_manager.Devices()[i].OpenDevice(ids_peak.DeviceAccessType_Control)
        node_map = device.RemoteDevice().NodeMaps()[0]
        break


# Construct the configuration folder path
config_folder = pathlib.Path("../../../OptiSort/HMI/Config").resolve()
ini_file = (config_folder / "ids_configuration.ini").resolve()

print(f"Resolved .pfs file path: {ini_file}")  # Output the resolved path for debugging

# Verify file existence and readability using pathlib
if not ini_file.is_file():
    raise FileNotFoundError(f"The file {ini_file} does not exist.")
if not os.access(ini_file, os.R_OK):  # os.access can still be used for checking readability
    raise PermissionError(f"The file {ini_file} is not readable.")

# Load the configuration from the .ini file
try:
    # Determine the current UEyeParametersetPath (str)
    value = node_map.FindNode("UEyeParametersetPath").Value()
    print(f'Current parameter path: {value}')
    # Set UEyeParametersetPath to "C:/settings/MyCamera" (str)
    node_map.FindNode("UEyeParametersetPath").SetValue(str(ini_file))
    # Execute UEyeParametersetLoad
    node_map.FindNode("UEyeParametersetLoad").Execute()
    # Check if the command has finished before you continue (optional)
    node_map.FindNode("UEyeParametersetLoad").WaitUntilDone()
    print(f"Configuration loaded successfully from {ini_file}")
except Exception as e:
    print(f"Failed to load configuration: {e}")


data_stream = device.DataStreams()
if data_stream.empty():
    raise RuntimeError("No Data Stream available")

data_stream = device.DataStreams()[0].OpenDataStream()
print(f"IDS camera initialized successfully! Using device: {device.DisplayName()}")
