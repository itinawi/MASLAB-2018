import cv2
import numpy as np;
from matplotlib import pyplot as plt
 
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
camera_capture = get_image()
cv2.imshow("Raw image", camera_capture)
filepath = "/home/maslab/Code/team1/team-1/image.png"
cv2.imwrite(filepath, camera_capture)
 
# release the camera
del(camera)

img = camera_capture
ret,thresh1 = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
ret,thresh2 = cv2.threshold(img,127,255,cv2.THRESH_BINARY_INV)
ret,thresh3 = cv2.threshold(img,127,255,cv2.THRESH_TRUNC)
ret,thresh4 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO)
ret,thresh5 = cv2.threshold(img,127,255,cv2.THRESH_TOZERO_INV)
titles = ['Original Image','BINARY','BINARY_INV','TRUNC','TOZERO','TOZERO_INV']
images = [img, thresh1, thresh2, thresh3, thresh4, thresh5]
for i in xrange(6):
    plt.subplot(2,3,i+1),plt.imshow(images[i],'gray')
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
plt.show()
