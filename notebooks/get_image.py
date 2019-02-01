import cv2

# Captures a single image from the camera and returns it in PIL format
def take_image():
    camera = cv2.VideoCapture(0)
	# take multiple pictures, because the first are very raw TODO tune the number later
    for i in range(5):
 	      # read is the easiest way to get a full image out of a VideoCapture object.
          retval, im = camera.read()
    del(camera)
    return im

def write_image(filepath):
    print("Taking image...")
    camera_frame = take_image()
    cv2.imshow("Raw image", camera_frame)
    cv2.imwrite(filepath, camera_frame)

