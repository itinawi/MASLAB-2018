import cv2
import sys
import datetime
import time

# Captures a single image from the camera and returns it in PIL format
def take_image(port_num):
    camera = cv2.VideoCapture(port_num)
	# take multiple pictures, because the first are very raw TODO tune the number later
    for i in range(10):
 	      # read is the easiest way to get a full image out of a VideoCapture object.
          retval, im = camera.read()
    del(camera)
    return im

def write_image(filepath, port_num):
    print("Taking image...")
    camera_frame = take_image(port_num)
    #cv2.imshow("Raw image", camera_frame)
    cv2.imwrite(filepath, camera_frame)


if __name__ == "__main__":
	if (len(sys.argv) == 1):
		ts = int(time.time())
		readable = datetime.datetime.fromtimestamp(ts).isoformat()
		filepath = "/home/maslab/Code/team1/src/opencv/images/" + str(readable) + ".jpg"
		print "No preferred filepath given. Writing image to ", filepath
	else:
		print "Writing to ", sys.argv[1]
		filepath = sys.argv[1]
  	write_image(filepath, 0)
