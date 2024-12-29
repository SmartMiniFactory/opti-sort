import cv2


class ImageProcessor:
    def __init__(self):
        pass

    def process_frame(self, frame):
        # OpenCV-based computation, e.g., object detection
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return processed_frame

    def get_results(self, processed_frame):
        # Return results (e.g., detected objects)
        pass