"""
This script was used to explore OpenCV library (image processing and feature recognition) by using random screenshots
of the target components to be found
"""

import cv2
import numpy as np
import keyboard


# Import two images
imgFront = cv2.imread("C:\\Users\\Davide Galli\\source\\repos\\opti-sort\\samples\\3D printed case\\Basler\\img1.bmp")
imgBack = cv2.imread("C:\\Users\\Davide Galli\\source\\repos\\opti-sort\\samples\\3D printed case\\Basler\\img2.bmp")

def rescaleFrame(frame, scale=0.5):
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dimensions = (width, height)
    return cv2.resize(frame, dimensions, interpolation=cv2.INTER_AREA)

scaledFront = rescaleFrame(imgFront)
scaledBack = rescaleFrame(imgBack)

cv2.imshow("Original front", scaledFront)

# method 1  (preferable) ---------------------------------

blurFront = cv2.GaussianBlur(scaledFront, (5,5), cv2.BORDER_DEFAULT)
blurBack = cv2.GaussianBlur(scaledBack, (5,5), cv2.BORDER_DEFAULT)
cv2.imshow("Blurred front", blurFront)

# Apply Canny edge detection to the images
cannyFront = cv2.Canny(scaledFront, 100, 100)
cannyBack = cv2.Canny(scaledBack, 100, 200)
cv2.imshow("Canny front", cannyFront)

# method 2 ---------------------------------

ret, thresh = cv2.threshold(scaledFront, 135, 255, cv2.THRESH_BINARY) # converts in binary blacks and whites
cv2.imshow("Thresholded front", thresh)

# detecting and drawing countours ----------

blankCanny = np.zeros(scaledFront.shape, dtype='uint8') # creating a blank image of the same size of scaledFront
blankThresh = np.zeros(scaledFront.shape, dtype='uint8') # creating a blank image of the same size of scaledFront

contoursCanny, hierarchiesCanny = cv2.findContours(cannyFront, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # detecting contours (canny)
contoursThresh, hierarchiesThresh = cv2.findContours(cannyFront, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE) # detecting contours (thresh)

cv2.drawContours(blankCanny, contoursCanny, -1, (0,0,255), 1)
cv2.imshow("Canny contours", blankCanny)
cv2.drawContours(blankThresh, contoursThresh, -1, (0,0,255), 1)
cv2.imshow("Thresh contours", blankThresh)


cv2.waitKey(0) # wait indefinitely

"""# Get only closed edges
edgesFront = cv2.morphologyEx(cannyFront, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)))
edgesBack = cv2.morphologyEx(cannyBack, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)))
cv2.imshow("Edges front", edgesFront)


# Recognise the features in the images
# Find the contours
contoursFront, _ = cv2.findContours(edgesFront, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contoursBack, _ = cv2.findContours(edgesBack, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


# Find triangles
trianglesFront = []
trianglesBack = []

for contour in contoursFront:
    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
    if len(approx) == 3:
        trianglesFront.append(approx)
    
for contour in contoursBack:
    perimeter = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
    if len(approx) == 3:
        trianglesBack.append(approx)

print("Press 'Q' to quit")

while True:

    # Display the images with contours
    cv2.drawContours(imgFront, contoursFront, -1, (0, 255, 0), 3)
    cv2.drawContours(imgBack, contoursBack, -1, (0, 255, 0), 3)
    
    # Display the images with triangles
    #cv2.drawContours(imgFront, trianglesFront, -1, (10, 0, 255), 3)
    #cv2.drawContours(imgBack, trianglesBack, -1, (10, 0, 255), 3)
    
    # Display the images with edges
    #cv2.imshow("Front Image with Edges", imgFront)
    #cv2.imshow("Back Image with Edges", imgBack)
    
    # Press q if you want to end the loop
    if cv2.waitKey(1) & keyboard.is_pressed('q'):
        print("\nQuitting...")
        break
"""

"""Being able to measure an object is possible with a mono camera, assuming the height of it does not change
The only thing to consider is that lenses present round edges (fisheye effect), shrinking/reducing distances at the edges
So there is the need for lens correction

Moreover, it's needed to calibrate a pixel to millimeter ratio, for example by picturing a ruler...
"""