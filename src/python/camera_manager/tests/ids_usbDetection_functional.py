import cv2
import numpy as np
from ids_peak import ids_peak
from ids_peak_ipl import ids_peak_ipl
from numpy.matlib import empty


# ----------- IDS Camera Wrapper (light version for simplicity) ------------

class IdsCamera:
    def __init__(self):
        ids_peak.Library.Initialize()
        self.device_manager = ids_peak.DeviceManager.Instance()
        self.device = None
        self.node_map = None
        self.data_stream = None

    def initialize(self):
        self.device_manager.Update()
        if self.device_manager.Devices().empty():
            raise RuntimeError("No IDS camera found")

        for dev in self.device_manager.Devices():
            if dev.IsOpenable():
                self.device = dev.OpenDevice(ids_peak.DeviceAccessType_Control)
                self.node_map = self.device.RemoteDevice().NodeMaps()[0]
                break
        if self.device is None:
            raise RuntimeError("No openable IDS camera found")

        self.data_stream = self.device.DataStreams()[0].OpenDataStream()

    def start_acquisition(self):
        self.data_stream.Flush(ids_peak.DataStreamFlushMode_DiscardAll)

        for buffer in self.data_stream.AnnouncedBuffers():
            self.data_stream.RevokeBuffer(buffer)

        payload_size = self.node_map.FindNode("PayloadSize").Value()
        num_buffers = self.data_stream.NumBuffersAnnouncedMinRequired()

        for _ in range(num_buffers):
            buffer = self.data_stream.AllocAndAnnounceBuffer(payload_size)
            self.data_stream.QueueBuffer(buffer)

        self.data_stream.StartAcquisition(ids_peak.AcquisitionStartMode_Default,
                                          ids_peak.DataStream.INFINITE_NUMBER)
        self.node_map.FindNode("TLParamsLocked").SetValue(1)
        self.node_map.FindNode("AcquisitionStart").Execute()

    def capture_frame(self):
        buffer = self.data_stream.WaitForFinishedBuffer(5000)
        if buffer is None:
            return None

        image = ids_peak_ipl.Image.CreateFromSizeAndBuffer(
            buffer.PixelFormat(),
            buffer.BasePtr(),
            buffer.Size(),
            buffer.Width(),
            buffer.Height()
        )

        img_array = image.get_numpy().astype(np.uint8)
        self.data_stream.QueueBuffer(buffer)

        return img_array

    def stop_acquisition(self):
        self.node_map.FindNode("AcquisitionStop").Execute()
        self.node_map.FindNode("TLParamsLocked").SetValue(0)
        self.data_stream.StopAcquisition()
        self.device.Close()


# ----------- Shape Detection Function -----------

import cv2
import numpy as np

def detect_shapes_and_classify(frame):
    # Convert to BGR if grayscale for visualization
    labeled_image = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if len(frame.shape) == 2 else frame.copy()

    # Convert to grayscale if input is BGR
    if len(frame.shape) == 3 and frame.shape[2] == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame.copy()

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Binary inverse threshold (black objects on white background)
    _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY_INV)

    # Morphological closing to clean small holes
    kernel = np.ones((3, 3), np.uint8)
    morphed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # Find external contours (components)
    contours, _ = cv2.findContours(morphed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected_objects = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        # print(f"Countour area - {area}")
        if area < 8000 or area > 20000:
            continue

        # Approximate contour
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

        # Default ROI contour color (red = invalid)
        roi_color = (0, 0, 255)

        shape_counts = {"triangle": 0, "square": 0, "rectangle": 0, "circle": 0}
        x = 0
        y = 0

        # Proceed if ROI is quadrilateral
        if len(approx) == 4:
            roi_color = (0, 255, 0)  # Green

            # Create mask for the ROI
            mask = np.zeros(thresh.shape, dtype=np.uint8)
            cv2.drawContours(mask, [cnt], -1, 255, -1)

            # Mask the threshold image
            roi_masked = cv2.bitwise_and(thresh, thresh, mask=mask)

            # Extract ROI region
            x, y, w, h = cv2.boundingRect(cnt)
            roi = roi_masked[y:y + h, x:x + w]

            # --- Optional transparent yellow fill for ROI ---
            overlay = labeled_image.copy()
            cv2.drawContours(overlay, [cnt], -1, (0, 255, 255), -1)  # Yellow fill
            alpha = 0.3
            cv2.addWeighted(overlay, alpha, labeled_image, 1 - alpha, 0, labeled_image)
            cv2.drawContours(labeled_image, [cnt], -1, (0, 255, 255), 2)

            # --- Marker detection inside ROI ---
            edges_roi = cv2.Canny(roi, 100, 200)
            markers, _ = cv2.findContours(edges_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for marker in markers:
                marker_area = cv2.contourArea(marker)
                if marker_area < 30:
                    continue

                marker_peri = cv2.arcLength(marker, True)
                approx_marker = cv2.approxPolyDP(marker, 0.025 * marker_peri, True)

                shape = None
                marker_color = (0, 0, 255)  # Default red (unidentified)

                if len(approx_marker) == 3:
                    shape = "triangle"
                    marker_color = (255, 0, 0)  # Blue
                elif len(approx_marker) == 4:
                    (mx, my, mw, mh) = cv2.boundingRect(approx_marker)
                    aspect_ratio = mw / float(mh)
                    if 0.9 < aspect_ratio < 1.1:
                        shape = "square"
                        marker_color = (0, 255, 255)  # Green
                    else:
                        shape = "rectangle"
                        marker_color = (255, 255, 0)  # Cyan
                else:
                    circularity = (4 * np.pi * marker_area) / (marker_peri * marker_peri)
                    if circularity > 0.7:
                        shape = "circle"
                        marker_color = (255, 0, 255)  # Magenta

                # print(shape, marker_area)

                if shape:
                    shape_counts[shape] = shape_counts.get(shape, 0) + 1

                    # Offset marker contour to global coordinates
                    marker_pts = marker + [x, y]
                    cv2.drawContours(labeled_image, [marker_pts], -1, marker_color, 2)

        # Always draw the ROI contour
        cv2.drawContours(labeled_image, [cnt], -1, roi_color, 2)

        # Classify component based on marker shapes
        label = "Unknown"

        if shape_counts == {"triangle": 2, "square": 0, "rectangle": 0, "circle": 0}:
            label = "AI"
        elif shape_counts == {"triangle": 1, "square": 1, "rectangle": 0, "circle": 0}:
            label = "AE"
        elif shape_counts == {"triangle": 0, "square": 0, "rectangle": 1, "circle": 1}:
            label = "BI"
        elif shape_counts == {"triangle": 0, "square": 0, "rectangle": 1, "circle": 2}:
            label = "BE"

        # Draw label above ROI rectangle
        cv2.putText(labeled_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        result = None

        # Draw orientated origin symbol for recognized components
        if label != "Unknown":
            # Get rotated rectangle
            rot_rect = cv2.minAreaRect(cnt)
            (center_x, center_y), (width, height), angle = rot_rect
            box = cv2.boxPoints(rot_rect)
            box = np.intp(box)

            result = {
                "component": label,
                "x": center_x,
                "y": center_y,
                "angle": angle
            }

            # Ensure width >= height (long edge = width)
            if width < height:
                width, height = height, width
                angle += 90

            # Compute all 4 sides (pairs of points)
            sides_all = [
                (box[0], box[1]),
                (box[1], box[2]),
                (box[2], box[3]),
                (box[3], box[0])
            ]

            # Compute side lengths and midpoints
            sides_info = []
            for p1, p2 in sides_all:
                length = np.hypot(p2[0] - p1[0], p2[1] - p1[1])
                mid_x = (p1[0] + p2[0]) / 2
                mid_y = (p1[1] + p2[1]) / 2
                sides_info.append({'p1': p1, 'p2': p2, 'length': length, 'mid': (mid_x, mid_y)})

            # Sort sides by length
            sides_info_sorted = sorted(sides_info, key=lambda s: s['length'])

            # Short sides = 2 shortest
            short_sides = sides_info_sorted[:2]

            # Among short sides, pick one whose midpoint is farthest from center (rounded side)
            distances = []
            for s in short_sides:
                mid_x, mid_y = s['mid']
                dist = np.hypot(mid_x - center_x, mid_y - center_y)
                distances.append(dist)

            rounded_idx = np.argmax(distances)  # Farthest midpoint
            p_rounded1 = short_sides[rounded_idx]['p1']
            p_rounded2 = short_sides[rounded_idx]['p2']

            # Compute vector of rounded short edge (p2 - p1)
            vec_rounded = np.array([p_rounded2[0] - p_rounded1[0], p_rounded2[1] - p_rounded1[1]])
            vec_rounded_unit = vec_rounded / np.linalg.norm(vec_rounded)

            # Compute normal (perpendicular)
            vec_normal_unit = np.array([-vec_rounded_unit[1], vec_rounded_unit[0]])

            # Y-axis = normal vector pointing AWAY from rounded edge (invert direction)
            vec_Y = -vec_normal_unit

            # X-axis = +90° rotation of Y (so clockwise wrt Y)
            vec_X = np.array([-vec_Y[1], vec_Y[0]])

            # Define axis lengths (relative to dimensions)
            Y_len = int(width * 0.15)  # 15% of long edge
            X_len = int(height * 0.15)  # 15% of short edge

            # Compute endpoints
            pt_Y = (int(center_x + vec_Y[0] * Y_len), int(center_y + vec_Y[1] * Y_len))
            pt_X = (int(center_x + vec_X[0] * X_len), int(center_y + vec_X[1] * X_len))

            # Draw arrows
            cv2.arrowedLine(labeled_image, (int(center_x), int(center_y)), pt_Y, (0, 0, 255), 2, tipLength=0.2)  # +Y
            cv2.arrowedLine(labeled_image, (int(center_x), int(center_y)), pt_X, (0, 0, 255), 2, tipLength=0.2)  # +X

        if result is not None:
            detected_objects.append(result)

    labels_colors = [
        ("Triangle", (255, 0, 0)),
        ("Square", (0, 255, 255)),
        ("Rectangle", (255, 255, 0)),
        ("Circle", (255, 0, 255)),
    ]

    # Add legend
    image_with_legend = add_legend(labeled_image, labels_colors)

    return thresh, image_with_legend, detected_objects


def add_legend(image, labels_colors, position=(10, 10), box_size=(20, 20), spacing=5, font_scale=0.6, thickness=1):

    x, y = position
    box_w, box_h = box_size
    font = cv2.FONT_HERSHEY_SIMPLEX

    for label, color in labels_colors:
        # Draw color box
        top_left = (x, y)
        bottom_right = (x + box_w, y + box_h)
        cv2.rectangle(image, top_left, bottom_right, color, -1)

        # Put text next to box
        text_x = x + box_w + spacing
        text_y = y + box_h - 5  # Align baseline
        cv2.putText(image, label, (text_x, text_y), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

        # Move down for next item
        y += box_h + spacing

    return image


# Buffer per le posizioni rilevate (per esempio, memorizziamo le posizioni per 10 cicli)
position_buffer = []
BUFFER_SIZE = 10

def stabilize_detection(result):
    # Aggiungi la posizione rilevata al buffer
    if result is not None:
        position_buffer.append((result["x"], result["y"], result["angle"]))

        # Mantieni solo i primi BUFFER_SIZE risultati
        if len(position_buffer) > BUFFER_SIZE:
            position_buffer.pop(0)

        # Calcola la posizione media se il buffer è abbastanza grande
        if len(position_buffer) == BUFFER_SIZE:
            avg_x = np.mean([pos[0] for pos in position_buffer])
            avg_y = np.mean([pos[1] for pos in position_buffer])
            avg_a = np.mean([pos[2] for pos in position_buffer])
            return avg_x, avg_y, avg_a
        else:
            return None  # Non abbastanza dati per una media stabile
    return None


def get_flexibowl_centre(image):

    # Convert to grayscale if input is BGR
    if len(frame.shape) == 3 and frame.shape[2] == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame.copy()

    # Step 1: Canny Edge Detection
    edges = cv2.Canny(gray, 50, 150)

    # Step 2: Thresholding (Otsu)
    _, binary = cv2.threshold(edges, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Step 3: Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    best_fit = None
    best_fit_score = 0

    for contour in contours:
        if len(contour) < 50:  # Filter out small contours
            continue

        if len(contour) >= 5:  # fitEllipse requires at least 5 points
            ellipse = cv2.fitEllipse(contour)
            (x, y), (MA, ma), angle = ellipse
            radius_estimate = (MA + ma) / 4  # approximate average radius

            if 50 < radius_estimate < 300:  # Radius filtering
                # Step 5: Evaluate fit quality
                ellipse_mask = np.zeros_like(gray)
                cv2.ellipse(ellipse_mask, ellipse, 255, 2)

                # Count how many contour points fall on the ellipse
                fit_score = 0
                for pt in contour:
                    px, py = pt[0]
                    if ellipse_mask[py, px] == 255:
                        fit_score += 1
                fit_ratio = fit_score / len(contour)

                # Update best match
                if fit_ratio > best_fit_score and fit_ratio > 0.6:  # only good fits
                    best_fit_score = fit_ratio
                    best_fit = ellipse

    # Step 6: Draw result
    if best_fit is not None:
        output = image.copy()
        (x, y), (MA, ma), angle = best_fit
        arc_center = (int(x), int(y))
        return output, best_fit, arc_center

    else:
        print("No arc detected with sufficient fit quality.")


# ----------- Main -----------

if __name__ == "__main__":
    # Initialize camera
    camera = IdsCamera()
    camera.initialize()
    camera.start_acquisition()

    while True:
        # Capture frame
        frame = camera.capture_frame()

        # Classify object and get labeled image
        thresh, labeled_image, detected_objects = detect_shapes_and_classify(frame)

        # è possibile che serva stabilizzare il risultato
        if len(detected_objects) > 0:
            print(detected_objects)

        # stabilizzazione
        """for result in detected_objects:
            # Stabilizza la posizione del componente
            stable_position = stabilize_detection(result)

            if stable_position is not None:
                # Mostra la posizione stabilizzata
                stable_x, stable_y = stable_position
                cv2.circle(labeled_image, (int(stable_x), int(stable_y)), 10, (0, 255, 0), -1)
                print(f"Stabilized position: ({stable_x}, {stable_y})")"""

        # Flexibowl center
        output, best_fit, flexi_center = get_flexibowl_centre(frame)
        cv2.ellipse(labeled_image, best_fit, (0, 255, 0), 2)

        print(f"Flexibowl's center {flexi_center}")

        # Show the thresholded and labeled images
        cv2.imshow("Thresholded", thresh)  # Uncomment if you want to see the thresholded image
        # cv2.imshow("Flexibowl center", output)
        cv2.imshow("Detected", labeled_image)

        # Exit condition (press 'q' to quit)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Stop camera acquisition after the loop finishes
    camera.stop_acquisition()
    cv2.destroyAllWindows()