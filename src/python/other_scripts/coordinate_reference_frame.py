import pathlib

import cv2
import numpy as np
import os
import yaml

def load_camera_calibration(calib_path):
    """Load camera calibration parameters from a .yaml file."""
    if not os.path.exists(calib_path):
        raise FileNotFoundError(f"Calibration path {calib_path} does not exist.")

    with open(calib_path, 'r') as file:
        data = yaml.safe_load(file)

    camera_matrix = np.array(data['camera_matrix'])
    distortion_coeffs = np.array(data['distortion_coefficients'])
    rotation_vectors = np.array(data['rotation_vectors'])
    translation_vectors = np.array(data['translation_vectors'])

    return camera_matrix, distortion_coeffs, rotation_vectors, translation_vectors


def undistort_image(image, camera_matrix, distortion_coeffs):
    """Undistort the input image using the calibration parameters."""
    h, w = image.shape[:2]
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, (w, h), 1, (w, h))
    undistorted = cv2.undistort(image, camera_matrix, distortion_coeffs, None, new_camera_matrix)
    return undistorted


def crop_image(image, x_start, y_start, width, height):
    """
    Crop the input image to a specified region of interest (ROI).

    Args:
        image (np.ndarray): Input image to be cropped.
        x_start (int): Starting x-coordinate of the crop.
        y_start (int): Starting y-coordinate of the crop.
        width (int): Width of the crop region.
        height (int): Height of the crop region.

    Returns:
        np.ndarray: Cropped image.
    """
    # Validate the ROI dimensions
    h, w = image.shape[:2]
    if x_start < 0 or y_start < 0 or x_start + width > w or y_start + height > h:
        raise ValueError("ROI is out of the image bounds.")

    # Crop the image using array slicing
    cropped = image[y_start:y_start + height, x_start:x_start + width]
    return cropped

def detect_circles(image):
    """Detect circles or arcs in the given image using Hough Transform."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)

    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,         # The accumulator resolution is slightly lower than the image resolution;
                        # Increasing it reduces computational cost but may miss smaller circles.
        minDist=10,     # Circles must be at least X pixels apart; Decrease this value if circles are closely packed.
        param1=50,      # Canny edge detection threshold.
        param2=70,      # Accumulator threshold for center detection.
                        # A higher value will result in fewer circles detected.
        minRadius=10,
        maxRadius=400,
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
    return circles


def find_inscribed_circle(circles):
    """Find the center of the circle inscribed between two other circles."""
    if circles is None or len(circles) < 2:
        raise ValueError("At least two circles are needed to calculate the inscribed circle.")

    circle1, circle2 = circles[:2]
    center1, center2 = circle1[:2], circle2[:2]
    radius1, radius2 = circle1[2], circle2[2]

    # Midpoint between the centers
    inscribed_center = ((center1[0] + center2[0]) // 2, (center1[1] + center2[1]) // 2)

    # Radius of the inscribed circle (approximation)
    inscribed_radius = abs(radius1 - radius2) // 2

    return inscribed_center, inscribed_radius


def overlay_debug_info(image, grid_size=50):
    """Overlay debug information on the image to help tune circle detection parameters.

    Args:
        image (np.ndarray): Input image (should be grayscale or BGR).
        grid_size (int): Size of each grid cell in pixels.

    Returns:
        np.ndarray: Image with overlaid debug information.
    """
    # Make a copy of the image to avoid modifying the original
    debug_image = image.copy()
    if len(debug_image.shape) == 2:  # If grayscale, convert to BGR for visualization
        debug_image = cv2.cvtColor(debug_image, cv2.COLOR_GRAY2BGR)

    # Get image dimensions
    h, w = debug_image.shape[:2]

    # Draw grid lines
    for x in range(0, w, grid_size):
        cv2.line(debug_image, (x, 0), (x, h), (0, 255, 0), 1)
    for y in range(0, h, grid_size):
        cv2.line(debug_image, (0, y), (w, y), (0, 255, 0), 1)

    # Calculate pixel intensity statistics for each grid cell
    for y in range(0, h, grid_size):
        for x in range(0, w, grid_size):
            # Extract the grid cell
            cell = image[y:y + grid_size, x:x + grid_size]
            mean_intensity = np.mean(cell)
            pixel_count = np.sum(cell > 0)  # Count non-zero pixels (useful for edges or features)

            # Write the stats on the grid cell
            text = f"Mean: {mean_intensity:.1f}\nPixels: {pixel_count}"
            org = (x + 5, y + 15)  # Text origin
            for i, line in enumerate(text.split("\n")):
                cv2.putText(debug_image, line, (org[0], org[1] + i * 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

    return debug_image


def main(image_path, calib_path):
    # Load the calibration files
    camera_matrix, distortion_coeffs, rvects, tvects = load_camera_calibration(calib_path)

    # Read and undistort the input image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Image {image_path} not found.")

    undistorted_image = undistort_image(image, camera_matrix, distortion_coeffs)

    # Get image dimensions
    height, width = undistorted_image.shape[:2]
    cropped_image = crop_image(undistorted_image, 0, 0, 450, height)

    # Generate the debug visualization
    # debug_image = overlay_debug_info(cropped_image, grid_size=100)
    # cv2.imshow("Debug Information", debug_image)

    # Detect circles in the AOI
    circles = detect_circles(cropped_image)
    if circles is not None:
        print(f"Detected circles: {circles}")

        # Find the inscribed circle's center and radius
        try:
            inscribed_center, inscribed_radius = find_inscribed_circle(circles)
            print(f"Inscribed circle center: {inscribed_center}, radius: {inscribed_radius}")

            # Draw the detected circles and the inscribed circle on the image
            for (x, y, r) in circles:
                cv2.circle(cropped_image, (x, y), r, (0, 255, 0), 2)

                # Annotate the radius near the circle
                text = f"r={r}px"
                cv2.putText(cropped_image, text, (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 1, cv2.LINE_AA)

            # cv2.circle(cropped_image, inscribed_center, inscribed_radius, (0, 0, 255), 2)


            # Show the result
            cv2.imshow("Detected Circles", cropped_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        except ValueError as e:
            print(e)
    else:
        print("No circles detected.")


if __name__ == "__main__":

    # Get the absolute path of the script directory
    script_dir = pathlib.Path(__file__).parent.resolve()
    image_path = script_dir / "../../OptiSort/HMI/Temp/luxonis_CalibrationImage_01.bmp"
    calib_path = script_dir / "../../OptiSort/HMI/config/luxonis_calibration.yaml"

    main(image_path, calib_path)
