"""
The script opens the camera, looks for the printed calibration grid and
exports a yaml file containing the lens distortion coefficients that are to be imported when doing object recognition
"""
import time

import keyboard
# ---------------------------------------------------------------------------------------------------------------------------------------

# Libraries
from pyueye import ueye
import numpy as np
import cv2
import yaml

# ---------------------------------------------------------------------------------------------------------------------------------------

hCam = ueye.HIDS(0)  # 0: first available camera;  1-254: The camera with the specified camera ID
sInfo = ueye.SENSORINFO()
cInfo = ueye.CAMINFO()
pcImageMemory = ueye.c_mem_p()
MemID = ueye.int()
rectAOI = ueye.IS_RECT()
pitch = ueye.INT()
nBitsPerPixel = ueye.INT(8)  # 24: bits per pixel for color mode; take 8 bits per pixel for monochrome
channels = 1  # 3: channels for color mode(RGB); take 1 channel for monochrome
m_nColorMode = ueye.INT()  # Y8/RGB16/RGB24/REG32
bytes_per_pixel = int(nBitsPerPixel / 8)

# ---------------------------------------------------------------------------------------------------------------------------------------
print("START")
print()

# Starts the driver and establishes the connection to the camera
nRet = ueye.is_InitCamera(hCam, None)
if nRet != ueye.IS_SUCCESS:
    print("is_InitCamera ERROR")

# Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
nRet = ueye.is_GetCameraInfo(hCam, cInfo)
if nRet != ueye.IS_SUCCESS:
    print("is_GetCameraInfo ERROR")

# You can query additional information about the sensor type used in the camera
nRet = ueye.is_GetSensorInfo(hCam, sInfo)
if nRet != ueye.IS_SUCCESS:
    print("is_GetSensorInfo ERROR")

nRet = ueye.is_ResetToDefault(hCam)
if nRet != ueye.IS_SUCCESS:
    print("is_ResetToDefault ERROR")

# Set display mode to DIB
nRet = ueye.is_SetDisplayMode(hCam, ueye.IS_SET_DM_DIB)

# Set the right color mode
if int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_BAYER:
    # set up the color depth to the current windows setting
    ueye.is_GetColorDepth(hCam, nBitsPerPixel, m_nColorMode)
    bytes_per_pixel = int(nBitsPerPixel / 8)
    print("IS_COLORMODE_BAYER: ", )
    print("\tm_nColorMode: \t\t", m_nColorMode)
    print("\tnBitsPerPixel: \t\t", nBitsPerPixel)
    print("\tbytes_per_pixel: \t\t", bytes_per_pixel)
    print()

elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_CBYCRY:
    # for color camera models use RGB32 mode
    m_nColorMode = ueye.IS_CM_BGRA8_PACKED
    nBitsPerPixel = ueye.INT(32)
    bytes_per_pixel = int(nBitsPerPixel / 8)
    print("IS_COLORMODE_CBYCRY: ", )
    print("\tm_nColorMode: \t\t", m_nColorMode)
    print("\tnBitsPerPixel: \t\t", nBitsPerPixel)
    print("\tbytes_per_pixel: \t\t", bytes_per_pixel)
    print()

elif int.from_bytes(sInfo.nColorMode.value, byteorder='big') == ueye.IS_COLORMODE_MONOCHROME:
    # for color camera models use RGB32 mode
    m_nColorMode = ueye.IS_CM_MONO8
    nBitsPerPixel = ueye.INT(8)
    bytes_per_pixel = int(nBitsPerPixel / 8)
    print("IS_COLORMODE_MONOCHROME: ", )
    print("\tm_nColorMode: \t\t", m_nColorMode)
    print("\tnBitsPerPixel: \t\t", nBitsPerPixel)
    print("\tbytes_per_pixel: \t\t", bytes_per_pixel)
    print()

else:
    # for monochrome camera models use Y8 mode
    m_nColorMode = ueye.IS_CM_MONO8
    nBitsPerPixel = ueye.INT(8)
    bytes_per_pixel = int(nBitsPerPixel / 8)
    print("else")

# Can be used to set the size and position of an "area of interest"(AOI) within an image
nRet = ueye.is_AOI(hCam, ueye.IS_AOI_IMAGE_GET_AOI, rectAOI, ueye.sizeof(rectAOI))
if nRet != ueye.IS_SUCCESS:
    print("is_AOI ERROR")

width = rectAOI.s32Width
height = rectAOI.s32Height

# Prints out some information about the camera and the sensor
print("Camera model:\t\t", sInfo.strSensorName.decode('utf-8'))
print("Camera serial no.:\t", cInfo.SerNo.decode('utf-8'))
print("Maximum image width:\t", width)
print("Maximum image height:\t", height)
print()

# ---------------------------------------------------------------------------------------------------------------------------------------

# Allocates an image memory for an image having its dimensions defined by width and height and its color depth defined by nBitsPerPixel
nRet = ueye.is_AllocImageMem(hCam, width, height, nBitsPerPixel, pcImageMemory, MemID)
if nRet != ueye.IS_SUCCESS:
    print("is_AllocImageMem ERROR")
else:
    # Makes the specified image memory the active memory
    nRet = ueye.is_SetImageMem(hCam, pcImageMemory, MemID)
    if nRet != ueye.IS_SUCCESS:
        print("is_SetImageMem ERROR")
    else:
        # Set the desired color mode
        nRet = ueye.is_SetColorMode(hCam, m_nColorMode)

# Activates the camera's live video mode (free run mode)
nRet = ueye.is_CaptureVideo(hCam, ueye.IS_DONT_WAIT)
if nRet != ueye.IS_SUCCESS:
    print("is_CaptureVideo ERROR")

# Enables the queue mode for existing image memory sequences
nRet = ueye.is_InquireImageMem(hCam, pcImageMemory, MemID, width, height, nBitsPerPixel, pitch)
if nRet != ueye.IS_SUCCESS:
    print("is_InquireImageMem ERROR")
else:
    print("Press q to leave the program")



# ---------------------------------------------------------------------------------------------------------------------------------------
# CALIBRATION

# Define grid parameters
pattern_size = (4, 10)  # (rows, columns)

# Define the real-world object points
objp = []
rows, cols = pattern_size
spacing = 20  # Spacing between circles in millimeters (or any unit)
for i in range(rows):
    for j in range(cols):
        # Offset alternate rows for the asymmetric pattern
        x = j * spacing
        y = i * spacing + (j % 2) * (spacing / 2)
        objp.append((x, y, 0))
objp = np.array(objp, dtype=np.float32)

object_points = []  # 3D points in real-world space
image_points = []   # 2D points in image plane

# ---------------------------------------------------------------------------------------------------------------------------------------

collectedSample = 0
toBeCollected = 1

while nRet == ueye.IS_SUCCESS and collectedSample < 15:

    array = ueye.get_data(pcImageMemory, width, height, nBitsPerPixel, pitch, copy=False)
    # bytes_per_pixel = int(nBitsPerPixel / 8)
    frame = np.reshape(array, (height.value, width.value, bytes_per_pixel))
    frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

    # ---------------------------------------------------------------------------------------------------------------------------------------
    # Include image data processing here

    # Find the circle grid
    found, centers = cv2.findCirclesGrid(
        frame,
        pattern_size,
        flags=cv2.CALIB_CB_ASYMMETRIC_GRID
    )

    if found:
        if collectedSample < toBeCollected:
            # Draw detected centers
            vis_image = cv2.drawChessboardCorners(frame, pattern_size, centers, found)
            cv2.imshow('Calibration acquisition', vis_image)
            cv2.waitKey(500)

            # Ensure data types
            object_points.append(objp.copy())  # Ensure a new numpy array is added
            image_points.append(np.squeeze(centers).astype(np.float32))  # Flatten and convert to float32
            collectedSample = len(object_points)

            print("Sample collected (", collectedSample, "/ 14), press 'n' to acquire next")

        # user must physically move calibration grid and acknowledge program to proceed
        if cv2.waitKey(500) and keyboard.is_pressed('n') and (collectedSample == toBeCollected):
            toBeCollected += 1

    else:
        print("Circle grid not found.")
        cv2.imshow('Calibration acquisition', frame)
        cv2.waitKey(500)

    # ---------------------------------------------------------------------------------------------------------------------------------------

# ---------------------------------------------------------------------------------------------------------------------------------------

print("Concluding calibration process...")

ueye.is_FreeImageMem(hCam, pcImageMemory, MemID)
ueye.is_ExitCamera(hCam)
cv2.destroyAllWindows()

# Perform camera calibration after collecting points
if object_points and image_points:

    print("First object points shape:", object_points[0].shape if object_points else "N/A")
    print("First image points shape:", image_points[0].shape if image_points else "N/A")

    ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points, image_points, frame.shape[::-1], None, None
    )

    if ret:
        print("Calibration successful!")
        print("Camera Matrix:\n", camera_matrix)
        print("Distortion Coefficients:\n", distortion_coeffs)

        # Save calibration results to a file
        data = {
            'camera_matrix': np.asarray(camera_matrix).tolist(),
            'dist_coeff': np.asarray(distortion_coeffs).tolist()
        }
        with open("calibration.yaml", "w") as f:
            yaml.dump(data, f)
        print("Calibration data saved to 'calibration.yaml'")
    else:
        print("Calibration failed.")
else:
    print("No points collected. Calibration not possible.")

print()
print("END")