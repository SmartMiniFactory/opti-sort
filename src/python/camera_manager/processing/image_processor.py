import cv2
import numpy as np


class ImageProcessor:
    def __init__(self):

        self.position_buffers = {}  # Chiave: componente (es. 'AE'), Valore: lista di posizioni
        self.BUFFER_SIZE = 20  # higher = more restrictive
        self.TOLERANCE = 1.0  # Tolleranza massima in pixel (adatta questo valore dopo test)
        self.HYSTERESIS_DISTANCE_MM = 30.0  # distanza minima per triggerare un nuovo messaggio

        self.last_sent_positions = {
            'AE': None,
            'BI': None,
            'AI': None,
            'BE': None
        }

        self.metrics = {"mean_brightness": [], "median_brightness": [], "cnr": [], "sharpness": [],
                        "white_saturation": [], "black_saturation": [], "midtones": [], "illum_uniformity": [],
                        "hist_spread": [], "snr": []}

    def detect_shapes_and_classify(self, frame):
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
                    approx_marker = cv2.approxPolyDP(marker, 0.03 * marker_peri, True)

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
                cv2.arrowedLine(labeled_image, (int(center_x), int(center_y)), pt_Y, (0, 0, 255), 2,
                                tipLength=0.2)  # +Y
                cv2.arrowedLine(labeled_image, (int(center_x), int(center_y)), pt_X, (0, 0, 255), 2,
                                tipLength=0.2)  # +X

            if result is not None:
                detected_objects.append(result)

        labels_colors = [
            ("Triangle", (255, 0, 0)),
            ("Square", (0, 255, 255)),
            ("Rectangle", (255, 255, 0)),
            ("Circle", (255, 0, 255)),
        ]

        # Add legend
        image_with_legend = self._add_legend(labeled_image, labels_colors)

        return thresh, image_with_legend, detected_objects

    def _add_legend(self, image, labels_colors, position=(10, 10), box_size=(20, 20), spacing=5, font_scale=0.6, thickness=1):

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

    def compute_pixel_mm_scale(self, img, grid_size, square_size_mm, draw=True):
        """ Compute pixel/mm ratio from checkerboard grid and visualize origin + axes """
        pattern_size = grid_size  # (cols, rows)
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        objp *= square_size_mm

        # Find corners
        ret, corners = cv2.findChessboardCorners(img, pattern_size)
        if not ret:
            raise ValueError("Chessboard corners not found")

        # Refine corners
        if len(img.shape) == 3 and img.shape[2] == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img.copy()
        criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        corners_np = corners2.reshape(-1, 2)

        # === REORDER corners: always top-left to bottom-right ===
        cols, rows = pattern_size
        # Sort by Y first, then X (for row-wise ordering)
        sorted_idx = np.lexsort((corners_np[:, 0], corners_np[:, 1]))  # Sort by Y then X
        corners_ordered = corners_np[sorted_idx]

        # Reshape in grid and check row consistency
        corners_grid = corners_ordered.reshape((rows, cols, 2))  # Shape (rows, cols, 2)
        # Ensure each row is left-to-right (sort X in each row)
        for r in range(rows):
            row = corners_grid[r]
            row_sorted = row[np.argsort(row[:, 0])]
            corners_grid[r] = row_sorted

        corners_np = corners_grid.reshape(-1, 2)

        # Pixel distances
        dx_pix = np.linalg.norm(corners_np[0] - corners_np[1])
        dy_pix = np.linalg.norm(corners_np[0] - corners_np[cols])

        scale_x = square_size_mm / dx_pix
        scale_y = square_size_mm / dy_pix

        # debug
        # print(f"dx_pix (1 square X): {dx_pix:.2f} px")
        # print(f"dy_pix (1 square Y): {dy_pix:.2f} px")
        # print(f"Scale X: {scale_x:.4f} mm/px, Scale Y: {scale_y:.4f} mm/px")
        # print(f"Pixel square ratio dx/dy: {dx_pix / dy_pix:.3f}")

        # Compute top-left (origin) and center
        chessboard_origin_px = tuple(corners_np[0])
        chessboard_center_px = tuple(np.mean(corners_np, axis=0))

        # Optional drawing
        vis_img = None
        if draw:
            vis_img = img.copy()
            if len(vis_img.shape) == 2:
                vis_img = cv2.cvtColor(vis_img, cv2.COLOR_GRAY2BGR)

            # Draw all corners
            corners2_draw = corners_np.reshape(-1, 1, 2).astype(np.float32)
            cv2.drawChessboardCorners(vis_img, pattern_size, corners2_draw, ret)

            # Draw top-left origin (red)
            cv2.circle(vis_img, (int(chessboard_origin_px[0]), int(chessboard_origin_px[1])), 7, (0, 0, 255), -1)
            cv2.putText(vis_img, "Origin (0,0)", (int(chessboard_origin_px[0]) + 5, int(chessboard_origin_px[1]) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

            # Draw center (green)
            cv2.circle(vis_img, (int(chessboard_center_px[0]), int(chessboard_center_px[1])), 7, (0, 255, 0), -1)
            cv2.putText(vis_img, "Center", (int(chessboard_center_px[0]) + 5, int(chessboard_center_px[1]) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # Draw arrows showing X and Y directions
            corner0 = corners_np[0]
            corner_x = corners_np[1]
            corner_y = corners_np[cols]
            vec_x = corner_x - corner0
            vec_y = corner_y - corner0

            # Arrow X (blue)
            end_x = (int(corner0[0] + vec_x[0] * 2), int(corner0[1] + vec_x[1] * 2))
            cv2.arrowedLine(vis_img, (int(corner0[0]), int(corner0[1])), end_x, (255, 0, 0), 2, tipLength=0.2)
            cv2.putText(vis_img, "+X", end_x, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Arrow Y (cyan)
            end_y = (int(corner0[0] + vec_y[0] * 2), int(corner0[1] + vec_y[1] * 2))
            cv2.arrowedLine(vis_img, (int(corner0[0]), int(corner0[1])), end_y, (255, 255, 0), 2, tipLength=0.2)
            cv2.putText(vis_img, "+Y", end_y, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        return scale_x, scale_y, chessboard_origin_px, chessboard_center_px, vis_img

    def pixel_to_scara(self, pixel_point, chessboard_center_px, scara_chessboard_center_mm, scale_x, scale_y, yaw_deg=0,
                       apply_yaw=False):
        """
        Convert pixel point to SCARA coords using chessboard pixel center & scara center, with optional yaw rotation
        + yaw = counter-clockwise rotation / openCV standard
        + apply_yaw = boolean to decide whether to apply yaw rotation
        """

        # Compute pixel differences from chessboard center
        dx_px = pixel_point[0] - chessboard_center_px[0]
        dy_px = pixel_point[1] - chessboard_center_px[1]

        # Flip X axis if necessary (as before)
        dx_mm = -dx_px * scale_x
        dy_mm = dy_px * scale_y

        if apply_yaw:
            # Convert yaw angle to radians
            yaw_rad = np.deg2rad(yaw_deg)

            # Rotation matrix
            cos_yaw = np.cos(yaw_rad)
            sin_yaw = np.sin(yaw_rad)

            dx_mm_rot = cos_yaw * dx_mm - sin_yaw * dy_mm
            dy_mm_rot = sin_yaw * dx_mm + cos_yaw * dy_mm
        else:
            dx_mm_rot = dx_mm
            dy_mm_rot = dy_mm

        # Final SCARA coordinates
        X_scara = scara_chessboard_center_mm[0] + dx_mm_rot
        Y_scara = scara_chessboard_center_mm[1] + dy_mm_rot

        return (X_scara, Y_scara)

    def stabilize_detection(self, result):
        """
        Stabilizza la posizione rilevata usando una media mobile separata per tipo di componente.
        Restituisce la posizione media stabilizzata solo se abbastanza stabile.
        """
        if result is None or "component" not in result:
            return None

        component = result["component"]

        # Inizializza il buffer se non esiste ancora per questo componente
        if component not in self.position_buffers:
            self.position_buffers[component] = []

        buffer = self.position_buffers[component]
        buffer.append((result["x"], result["y"], result["angle"]))

        # Mantieni solo i primi BUFFER_SIZE risultati
        if len(buffer) > self.BUFFER_SIZE:
            buffer.pop(0)

        # Calcola la media e verifica la stabilità se il buffer è pieno
        if len(buffer) == self.BUFFER_SIZE:
            xs = [pos[0] for pos in buffer]
            ys = [pos[1] for pos in buffer]
            angles = [pos[2] for pos in buffer]

            avg_x = np.mean(xs)
            avg_y = np.mean(ys)
            avg_a = np.mean(angles)

            std_x = np.std(xs)
            std_y = np.std(ys)

            if std_x < self.TOLERANCE and std_y < self.TOLERANCE:
                return component, avg_x, avg_y, avg_a  # Ritorna anche il tipo di componente
            else:
                return None  # Posizione non ancora stabile
        else:
            return None

    def should_send_mqtt(self, component, current_scara_coords):
        """
        Determina se inviare il messaggio MQTT in base all'isteresi spaziale.
        """
        self.last_sent_positions
        last_pos = self.last_sent_positions.get(component)

        if last_pos is None:
            # Nessuna posizione inviata prima → invia subito
            self.last_sent_positions[component] = current_scara_coords
            return True

        # Calcola distanza euclidea
        dx = current_scara_coords[0] - last_pos[0]
        dy = current_scara_coords[1] - last_pos[1]
        distance = np.hypot(dx, dy)

        if distance >= self.HYSTERESIS_DISTANCE_MM:
            # Aggiorna la posizione e consenti l'invio
            last_sent_positions[component] = current_scara_coords
            return True
        else:
            return False

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

