import cv2
import numpy as np


def sample():
    imgTest = cv2.imread("C:\\Users\\dylan\\Documents\\OpenCVTestShapes.png")

    # converting image into grayscale image
    gray = cv2.cvtColor(imgTest, cv2.COLOR_BGR2GRAY)

    # setting threshold of gray image
    _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # using a findContours() function
    contours, hierarchy = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    i = 0

    # show hierarchy structure
    """for h1 in hierarchy:
        for h in h1:
            print("Contour", i, ": next", h[0], "; previous", h[1], "first child", h[2], "; parent", h[3])
            i += 1"""

    i = 0
    for contour in contours:

        # skipping first cycle because findcontour function detects whole image as shape
        if i == 0:
            i = 1
            continue

        # cv2.approxPloyDP() function to approximate the shape
        approx = cv2.approxPolyDP(
            contour, 0.01 * cv2.arcLength(contour, True), True)

        # using drawContours() function
        cv2.drawContours(imgTest, [contour], 0, (0, 0, 255), 2)

        # finding center point of shape
        M = cv2.moments(contour)
        if M['m00'] != 0.0:
            x = int(M['m10'] / M['m00'])
            y = int(M['m01'] / M['m00'])

        # detecting hexagons (outer USB contours) and parent is 0 (image)
        if len(approx) == 6 and hierarchy[0][i][3] == 0:

            triangle_counter = 0
            quadrilateral_counter = 0
            circle_counter = 0

            child_index = hierarchy[0][i][2]
            while child_index != -1:

                approx = cv2.approxPolyDP(
                    contours[int(child_index)], 0.01 * cv2.arcLength(contours[int(child_index)], True), True)

                if len(approx) == 3:
                    triangle_counter += 1

                elif len(approx) == 4:
                    quadrilateral_counter += 1

                elif len(approx) == 5:
                    quadrilateral_counter += 1

                else:
                    circle_counter += 1

                child_index = hierarchy[0][child_index][0]  # Next sibling

            if triangle_counter == 2 and quadrilateral_counter == 0 and circle_counter == 0:
                cv2.putText(imgTest, "Component A - inside", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 200, 0), 2)

            if triangle_counter == 1 and quadrilateral_counter == 1 and circle_counter == 0:
                cv2.putText(imgTest, "Component A - outside", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 200, 0), 2)

            if triangle_counter == 0 and quadrilateral_counter == 1 and circle_counter == 2:
                cv2.putText(imgTest, "Component B - outside", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 200, 0), 2)

            if triangle_counter == 0 and quadrilateral_counter == 1 and circle_counter == 1:
                cv2.putText(imgTest, "Component B - inside", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 200, 0), 2)

        i += 1

    # displaying the image after drawing contours
    cv2.imshow('Test shapes', imgTest)

def main():
    # Open the webcam
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply binary thresholding
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

        # Perform morphological operations to close gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        morphed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # Perform edge detection using Canny
        edges = cv2.Canny(morphed, 50, 150)
        cv2.imshow('Edges', edges)

        # Find contours in the edged image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Approximate the contour to simplify its shape
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the contour has 3 vertices (triangle)
            if len(approx) == 3:
                # Draw the triangle
                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 3)

        # Display the resulting frame
        cv2.imshow('Triangle Detection', frame)

        sample()

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()