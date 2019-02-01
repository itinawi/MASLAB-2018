import cv2
import numpy as np
import sys
 
# define our camera to be the default
camera = cv2.VideoCapture(0)
 
# Captures a single image from the camera and returns it in PIL format
def get_image():
	# take multiple pictures, because the first are very raw TODO tune the number later
	for i in range(10):
 		# read is the easiest way to get a full image out of a VideoCapture object.
		retval, im = camera.read()
	return im
 
print("Taking image...")
# Take the actual image we want to keep and save it locally
camera_capture = get_image()
cv2.imshow("Raw image", camera_capture)
filepath = "/home/maslab/opencv/image.png"
cv2.imwrite(filepath, camera_capture)
# release the camera
del(camera)
 
if len(sys.argv) > 1:
	filepath = sys.argv[1]
else:
	#filepath = "/home/maslab/Code/team1/team-1/image.png"
  filepath = "/home/maslab/opencv/image.png"

print "working with image from ", filepath
im = cv2.imread(filepath, 0)
cv2.imshow("Unprocessed", im)

#################################################################################################
# Set up the detector with custom parameters.
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
#params.minThreshold = 10
#params.maxThreshold = 200

# Filter by Area.
#params.filterByArea = True
params.minArea = 100
params.maxArea = 1000

# Filter by Circularity
#params.filterByCircularity = True
params.minCircularity = 0.95
params.maxCircularity = 1

# Filter by Convexity
#params.filterByConvexity = True
#params.minConvexity = 0.87

# Filter by Inertia
#params.filterByInertia = True
#params.minInertiaRatio = 0.01

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector(params)

#################################################################################################

# Detect blobs.
keypoints = detector.detect(im)
print "number of detected blobs ", len(keypoints)
 
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)
