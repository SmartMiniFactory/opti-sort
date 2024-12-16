import cv2


def prepare_image(frame, sample):
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if sample:
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        return thresh

    else:
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

    # Perform morphological operations to close gaps
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    morphed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # Perform edge detection using Canny
    edges = cv2.Canny(morphed, 50, 150)

    return edges


def detect_shape(frame):
    contours, hierarchy = cv2.findContours(frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    i = 0

    # show hierarchy structure
    """
    for h1 in hierarchy:
        for h in h1:
            print("Contour", i, ": next", h[0], "; previous", h[1], "first child", h[2], "; parent", h[3])
            i += 1

    i = 0 # reset index
    """

    detected_shapes = {
        "A": {
            "inside": {
                "center": [],
                "contour": []
            },
            "outside": {
                "center": [],
                "contour": []
            },
        },
        "B": {
            "inside": {
                "center": [],
                "contour": []
            },
            "outside": {
                "center": [],
                "contour": []
            }
        }
    }

    for contour in contours:
        # skipping first cycle because findContours() function detects whole image as shape
        if i == 0:
            i = 1
            continue

        # approximate the shape
        approx_value = 0.01
        approx = cv2.approxPolyDP(
            contour, approx_value * cv2.arcLength(contour, True), True)

        cv2.drawContours(frame, [contour], 0, (0, 0, 255), 5)

        # detecting hexagons (outer USB contours) and parent is 0 (image)
        if len(approx) == 6 and hierarchy[0][i][3] == 0:

            # finding center point of shape
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10'] / M['m00'])
                y = int(M['m01'] / M['m00'])

            triangle_counter = 0
            quadrilateral_counter = 0
            circle_counter = 0

            # analyzing hexagon's children and detect their shapes
            child_index = hierarchy[0][i][2]
            while child_index != -1:

                # approximating contour of specific child
                approx = cv2.approxPolyDP(
                    contours[int(child_index)], approx_value * cv2.arcLength(contours[int(child_index)], True), True)

                if len(approx) == 3:
                    triangle_counter += 1

                elif len(approx) == 4:
                    quadrilateral_counter += 1

                elif len(approx) == 5:
                    quadrilateral_counter += 1

                else:
                    circle_counter += 1

                child_index = hierarchy[0][child_index][0]  # Next sibling

            print(triangle_counter, quadrilateral_counter, circle_counter)

            # count hexagon's children shapes to associate component to detect
            if triangle_counter == 2 and quadrilateral_counter == 0 and circle_counter == 0:
                print("a inside ok")
                detected_shapes["A"]["inside"]["center"].append(x)
                detected_shapes["A"]["inside"]["center"].append(y)
                detected_shapes["A"]["inside"]["contour"].append(contour)

            if triangle_counter == 1 and quadrilateral_counter == 1 and circle_counter == 0:
                detected_shapes["A"]["outside"]["center"].append(x)
                detected_shapes["A"]["outside"]["center"].append(y)
                detected_shapes["A"]["outside"]["contour"].append(contour)

            if triangle_counter == 0 and quadrilateral_counter == 1 and circle_counter == 2:
                detected_shapes["B"]["inside"]["center"].append(x)
                detected_shapes["B"]["inside"]["center"].append(y)
                detected_shapes["B"]["inside"]["contour"].append(contour)

            if triangle_counter == 0 and quadrilateral_counter == 1 and circle_counter == 1:
                detected_shapes["B"]["outside"]["center"].append(x)
                detected_shapes["B"]["outside"]["center"].append(y)
                detected_shapes["B"]["outside"]["contour"].append(contour)

        i += 1
    return detected_shapes  # as [coordinates of the component's center, component's outer contour]


def draw_contour(original_frame, components):
    for component_type in components:  # A or B
        for orientation in components[component_type]:  # inside or outside

            # check if at least a center per component is detected
            if len(components[component_type][orientation]["center"]) == 0:
                continue

            text = "Component " + component_type + " - " + orientation
            contour = components[component_type][orientation]["contour"]
            x = components[component_type][orientation]["center"][0]
            y = components[component_type][orientation]["center"][1]
            cv2.drawContours(original_frame, contour, 0, (0, 0, 255), 2)
            cv2.putText(original_frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 200, 0), 2)
    return original_frame


def main():
    # Open the webcam
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        # Video frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        """
        # rule-based algorithm test
        ideal_shapes = cv2.imread("C:\\Users\\dylan\\Documents\\OpenCVTestShapes.png")
        detected_components = detect_shape(prepare_image(ideal_shapes, True))
        modified_frame = draw_contour(ideal_shapes, detected_components)
        cv2.imshow('Rule-based algorithm test on ideal shapes', modified_frame)
        """

        # camera acquisition
        """
        prep = prepare_image(frame, False)
        cont, h = cv2.findContours(prep, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in cont:
            approx = cv2.approxPolyDP(c, 0.01 * cv2.arcLength(c, True), True)
            cv2.drawContours(frame, [c], 0, (0, 0, 255), 2)

        cv2.imshow('Shape AAA', frame)

        detected = detect_shape(prep)
        # modified = draw_contour(frame, detected)
        # cv2.imshow('Shape detection on camera stream', modified)
        """

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
