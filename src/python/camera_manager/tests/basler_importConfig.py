import os
from pypylon import pylon
import cv2
import keyboard
import pathlib

baslerCamera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

# Grabing Continusely (video) with minimal delay
baslerCamera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
baslerConverter = pylon.ImageFormatConverter()

# Converting to opencv bgr format
baslerConverter.OutputPixelFormat = pylon.PixelType_BGR8packed
baslerConverter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# Construct the configuration folder path
config_folder = pathlib.Path("../../../OptiSort/HMI/Config").resolve()
pfs_file = (config_folder / "basler_configuration.pfs").resolve()

print(f"Resolved .pfs file path: {pfs_file}")  # Output the resolved path for debugging

# Verify file existence and readability using pathlib
if not pfs_file.is_file():
    raise FileNotFoundError(f"The file {pfs_file} does not exist.")
if not os.access(pfs_file, os.R_OK):  # os.access can still be used for checking readability
    raise PermissionError(f"The file {pfs_file} is not readable.")

baslerCamera.Open()
node_map = baslerCamera.GetNodeMap()
pylon.FeaturePersistence.Load(str(pfs_file), baslerCamera.GetNodeMap(), True)

while True:

    frame = None

    grabResult = baslerCamera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

    if grabResult.GrabSucceeded():
        # Access the image data
        image = baslerConverter.Convert(grabResult)
        img = image.GetArray()
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

    grabResult.Release()

    # Show the frame
    cv2.imshow("Basler Camera Stream", frame)


    # Press q if you want to end the loop
    if cv2.waitKey(1) & keyboard.is_pressed('q'):
        print("\nQuitting...")
        break


baslerCamera.StopGrabbing()
baslerCamera.Close()
cv2.destroyAllWindows()