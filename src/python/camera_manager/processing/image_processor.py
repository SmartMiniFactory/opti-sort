import cv2
import numpy as np


class ImageProcessor:
    def __init__(self):
        self.metrics = {}
        self.metrics["mean_brightness"] = []
        self.metrics["median_brightness"] = []
        self.metrics["cnr"] = []
        self.metrics["sharpness"] = []
        self.metrics["white_saturation"] = []
        self.metrics["black_saturation"] = []
        self.metrics["midtones"] = []
        self.metrics["illum_uniformity"] = []
        self.metrics["hist_spread"] = []
        self.metrics["snr"] = []


    def process_frame(self, frame):
        # OpenCV-based computation, e.g., object detection
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return processed_frame

    def calculate_image_quality(self, frame):

        # MEAN/MEDIAN BRIGHTNESS
        # Measures the overall light intensity of the image; Useful for detecting under/overexposure
        mean_brightness = np.mean(frame)
        self.metrics["mean_brightness"].append(mean_brightness)
        self.metrics["median_brightness"].append(np.median(frame))

        # CONTRAST-TO-NOISE RATIO (CNR)
        # Measures how clearly the object (signal) stands out from the background (noise).
        # High CNR usually means better image quality for detection.
        #signal_mean = np.mean(roi)  # region of interest
        #background_std = np.std(background)  # outside the region of interest
        #self.metrics["cnr"] = ((signal_mean - np.mean(background)) / (background_std + 1e-6)).append()

        # SHARPNESS / FOCUS MEASURE
        # Indicates how in-focus an image is. Blurry images = bad detection.
        laplacian = cv2.Laplacian(frame, cv2.CV_64F)
        self.metrics["sharpness"].append(laplacian.var())

        # WHITE LEVEL SATURATION
        # Measures how close the image is to saturation (clipping at white).
        # Overexposed images will have many pixels near 255
        self.metrics["white_saturation"].append(np.sum(frame >= 250) / frame.size)  # % of nearly white pixels

        # BLACK LEVEL SATURATION
        self.metrics["black_saturation"].append(np.sum(frame <= 5) / frame.size)  # % of nearly black pixels

        # MIDTONE DISTRIBUTION
        # Checks how much of the image is concentrated in the midtones (good exposure).
        # A healthy image should have a balanced midtone presence unless it's a high-contrast scene
        self.metrics["midtones"].append(np.sum((frame > 50) & (frame < 200)) / frame.size)

        # ILLUMINATION UNIFORMITY
        # Measures consistency of brightness across the image
        # Uneven lighting (e.g., shadows or hotspots) can mess with detection
        # Lower standard deviation = more uniform lighting.
        h, w = frame.shape[:2]
        tiles = [frame[y:y + h // 3, x:x + w // 3] for y in range(0, h, h // 3) for x in range(0, w, w // 3)]
        means = [np.mean(tile) for tile in tiles]
        self.metrics["illum_uniformity"].append(np.std(means))

        # HISTOGRAM SPREAD / EXPOSURE RANGE
        self.metrics["hist_spread"].append(np.std(frame))

        # SNR (Signal-to-Noise Ratio)
        # Compares the strength of useful signal (image) to background noise. High SNR = cleaner image.
        self.metrics["snr"].append(mean_brightness / (np.std(frame) + 1e-6))

    def get_metrics(self):
        averages = {}

        # Loop through each metric in self.metrics
        for metric, values in self.metrics.items():
            if values:  # Only calculate the average if there are values in the list
                averages[metric] = sum(values) / len(values)
            else:
                averages[metric] = None  # No data to average

        return averages

