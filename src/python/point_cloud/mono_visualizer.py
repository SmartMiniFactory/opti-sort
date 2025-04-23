#!/usr/bin/env python3

import depthai as dai
import numpy as np
import cv2

pipeline = dai.Pipeline()


# Define sources and outputs
monoL = pipeline.create(dai.node.MonoCamera)
monoL.setCamera("left")
monoL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)


# Linking ---------------------------------
monoOutLeft = pipeline.create(dai.node.XLinkOut)
monoOutLeft.setStreamName("monoLeft")
monoL.out.link(monoOutLeft.input)

with dai.Device(pipeline) as device:

    qLeft = device.getOutputQueue(name="monoLeft", maxSize=4, blocking=False)

    while True:
        if qLeft.has():
            # Load the image
            gray = qLeft.get().getCvFrame()

            # Blur to reduce noise and improve edge detection
            gray_blurred = cv2.medianBlur(gray, 5)

            # Detect circles using Hough Circle Transform
            circles = cv2.HoughCircles(
                gray_blurred,
                cv2.HOUGH_GRADIENT,
                dp=1.2,
                minDist=50,
                param1=100,
                param2=30,
                minRadius=20,
                maxRadius=0
            )
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for i in circles[0, :1]:  # Take only the first detected circle
                    center_x, center_y, radius = i

                    # Draw the circle and its center (for visualization)
                    cv2.circle(gray, (center_x, center_y), radius, (0, 255, 0), 2)
                    cv2.circle(gray, (center_x, center_y), 2, (0, 0, 255), 3)

                    # Define ROI using bounding box
                    x1 = max(center_x - radius, 0)
                    y1 = max(center_y - radius, 0)
                    x2 = min(center_x + radius, gray.shape[1])
                    y2 = min(center_y + radius, gray.shape[0])

                    roi = gray[y1:y2, x1:x2]

                    # Show or save ROI
                    cv2.imshow("Detected ROI", roi)
                    cv2.imshow("Detected Circle", gray)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
            else:
                print("No circles were detected.")

            #cv2.imshow("Mono Left", qLeft.get().getCvFrame())

        if cv2.waitKey(1) == ord('q'):
            break


