"""The C# program prepares in a folder 45 images in total (15 per camera) containing
the calibration grids. This script is supposed to cycle through them and perform the camera calibration for all three
cameras. The script fails if:
- not enough images (less than 15 per camera) are provided
- bad images are provided (grid not found)
- cv2 library fail
In case of failure, the calibration file is not exported. In case of bad image, the C# program will delete it and ask
the user to retake it."""
# ---------------------------------------------------------------------------------------------------------------------------------------
import json
import os
import cv2
import yaml
import numpy as np
import pathlib
import glob
# ---------------------------------------------------------------------------------------------------------------------------------------
# GRID PARAMETERS

# vertex
cols = 5
rows = 7

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((cols * rows, 3), np.float32)
objp[:, :2] = np.mgrid[0:rows, 0:cols].T.reshape(-1, 2)

object_points = []  # 3D points in real-world space
image_points = []  # 2D points in image plane

images = glob.glob('*.jpg')

# Get the absolute path of the script directory
script_dir = pathlib.Path(__file__).parent.resolve()

# Define the Temp folder path relative to the script directory
temp_folder = script_dir / "../../OptiSort/HMI/Temp"
config_folder = script_dir / "../../OptiSort/HMI/Config"

# ---------------------------------------------------------------------------------------------------------------------------------------

result = {
    "bad_image": {
        "ids": [],
        "basler": [],
        "luxonis": []
    },
    "calibration_fail": {
        "ids": False,
        "basler": False,
        "luxonis": False
    }
}

for camera in ["ids", "basler", "luxonis"]:
    for picture_nr in range(15):

        # get picture path
        filename = f"{camera}_CalibrationImage_{picture_nr + 1:02d}" # using format 01, 02, ... for digits
        absolutePath = (temp_folder / f"{filename}.bmp").resolve()

        # check picture existence
        if absolutePath.exists():

            # acquire image with openCV
            image = cv2.imread(str(absolutePath))
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (rows, cols), None)

            # chess board found
            if ret:

                object_points.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                image_points.append(corners2)
                cv2.drawChessboardCorners(image, (rows, cols), corners2, ret)

                # saving image with grid (cool but unuseful)
                # cv2.imwrite(str(temp_folder / f"{filename}_grid.bmp"), image)

            # chess board not found
            else:
                result["bad_image"][camera].append(picture_nr + 1)
                # print(f"calibration grid NOT found for {filename}") # debug string deactivated to not confuse C#

        else:

            result["bad_image"][camera].append(picture_nr + 1)
            # print(f"{filename} does not exist") # debug string deactivated to not confuse C#

    # CALIBRATION -------------------------------------------------------

    # chess board should be found in all the pictures for that camera
    if not result["bad_image"][camera]:

        # check if points are collected
        if object_points and image_points:
            ret, mtx, dist, rvecs, tvecs = (
                cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None)
            )

            if ret:
                # debug string deactivated to not confuse C#
                # print("Calibration successful!")
                # print("Camera Matrix:\n", mtx)
                # print("Distortion Coefficients:\n", dist)
                # print("Rotation vectors:\n", rvecs)
                # print("Translation vectors:\n", tvecs)

                # Save calibration results to a file
                data = {
                    'camera_matrix': np.asarray(mtx).tolist(),
                    'distortion_coefficients': np.asarray(dist).tolist(),
                    'rotation_vectors': np.asarray(rvecs).tolist(),
                    'translation_vectors': np.asarray(tvecs).tolist()
                }

                with open(config_folder / f"{camera}_calibration.yaml", "w") as f:
                    yaml.dump(data, f)
                    # print("Calibration data saved to 'calibration.yaml'") # debug string deactivated to not confuse C#

            else:
                result["calibration_fail"][camera] = True

print(json.dumps(result))
