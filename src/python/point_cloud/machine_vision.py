import cv2
import numpy as np

class OptiSortVision():
    def __init__(self):
        pass
    def show_frame(self, image):
        # Preprocess image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Image", gray)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def get_flexibowl_centre(self, image):

        # Preprocess image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

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

            cv2.ellipse(output, best_fit, (0, 255, 0), 2)
            cv2.circle(output, arc_center, 5, (0, 0, 255), -1)  # Draw center
            print(f"Arc center: {arc_center}")
            cv2.imshow("Detected Arc", output)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        else:
            print("No arc detected with sufficient fit quality.")

    def get_centre(self, image):

        # Preprocess image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

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

                if 150 < radius_estimate < 500:  # Radius filtering
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

            cv2.ellipse(output, best_fit, (0, 255, 0), 2)
            cv2.circle(output, arc_center, 5, (0, 0, 255), -1)  # Draw center
            print(f"Arc center: {arc_center}")
            cv2.imshow("Detected Arc", output)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        else:
            print("No arc detected with sufficient fit quality.")


