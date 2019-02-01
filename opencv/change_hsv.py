import cv2
from PIL import Image

def change_hsv(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #convert it to hsv

	h, s, v = cv2.split(hsv)
	v += 10000
	final_hsv = cv2.merge((h, s, v))

	img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
	Image.fromarray(np.array(img))
	return img
