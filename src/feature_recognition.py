import cv2
import keyboard


# Import the images
imgFront = cv2.imread("C:\\Users\\DGalli\\source\\repos\\opti-sort\\samples\\3D printed case\\Basler\\img1.bmp")
imgBack = cv2.imread("C:\\Users\\DGalli\\source\\repos\\opti-sort\\samples\\3D printed case\\Basler\\img2.bmp")

# Convert the images to grayscale
imgFront = cv2.cvtColor(imgFront, cv2.COLOR_BGR2GRAY)
imgBack = cv2.cvtColor(imgBack, cv2.COLOR_BGR2GRAY)

# Apply Canny edge detection to the images
edgesFront = cv2.Canny(imgFront, 100, 200)
edgesBack = cv2.Canny(imgBack, 100, 200)

# Get only closed edges
edgesFront = cv2.morphologyEx(edgesFront, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)))
edgesBack = cv2.morphologyEx(edgesBack, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (5,5)))

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
    cv2.imshow("Front Image with Edges", imgFront)
    cv2.imshow("Back Image with Edges", imgBack)
    
    # Press q if you want to end the loop
    if cv2.waitKey(1) & keyboard.is_pressed('q'):
        print("\nQuitting...")
        break
