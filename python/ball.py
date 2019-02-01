import cv2
import numpy as np
from PIL import Image, ImageFilter

MIN_SATURATION = 50
IMAGE_WIDTH = 400
IMAGE_HEIGHT = 300
BLUR_RADIUS = 5
BALL_SANITY_SLOPE = 6.5
BALL_SANITY_OFFSET = 0.25
ERROR_THRESHOLD = 0.1
MIN_HEIGHT = 0.25

def sanity_check(keypoint):
  y_value = keypoint.pt[1] / IMAGE_HEIGHT
  radius = keypoint.size / (2.0 * IMAGE_HEIGHT)
  error = abs(y_value - (radius * BALL_SANITY_SLOPE + BALL_SANITY_OFFSET))
  return error < ERROR_THRESHOLD and y_value > MIN_HEIGHT

def get_closest_ball(img, show=False):
  keypoints = get_keypoints(img, show=show)
  return get_closest_keypoint(keypoints)
    
def get_closest_keypoint(keypoints):
  if len(keypoints) == 0:
    return (0, 0), 0
  x_mid = IMAGE_WIDTH / 2
  y_bot = IMAGE_HEIGHT
  min_dist = int(1e10)
  closest = (0, 0), 0
  for keypoint in keypoints:
    x_distance = (keypoint.pt[0] - x_mid) ** 2
    y_distance = (keypoint.pt[1] - y_bot) ** 2
    distance = x_distance + y_distance
    if distance < min_dist:
      min_dist = distance
      closest = keypoint.pt, keypoint.size
  return closest

def get_keypoints(img, show=False):
  img = img.filter(ImageFilter.GaussianBlur(BLUR_RADIUS))
  saturation = np.array(img.convert("HSV"))[:, :, 1]
  mask = ((saturation > MIN_SATURATION) * 255).astype(np.uint8)
  # Set the parameters
  params = cv2.SimpleBlobDetector_Params()
  params.filterByCircularity = False
  params.minCircularity = 0.7
  params.maxCircularity = 1
  params.filterByColor = True
  params.blobColor = 255
  # Create a detector and detect
  detector = cv2.SimpleBlobDetector(params)
  keypoints = detector.detect(mask)
  print "Detected {} blobs!".format(len(keypoints))
  # keypoints = filter(sanity_check, keypoints)
  print "Filtered {} blobs!".format(len(keypoints))
  # Display the detections
  if show:
    vis = cv2.drawKeypoints(mask, 
                            keypoints, 
                            np.array([]),
                            (255, 0, 0), 
                            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    Image.fromarray(vis).show()
  return keypoints

if __name__ == "__main__":
  img = Image.open("/Users/KOCABEY/Downloads/Picture 18.jpg")
  img = img.resize((IMAGE_WIDTH, IMAGE_HEIGHT))
  keypoint = get_closest_ball(img, show=True)
  print keypoint
