import cv2
import math
import numpy as np
from PIL import Image, ImageFilter

WHITE_THRESHOLD = 160
SATURATION_THRESHOLD = 50
HUE_LOWER_THRESHOLD = 180
HUE_HIGHER_THRESHOLD = 255
BLUR_RADIUS = 5

IMAGE_WIDTH = 400
IMAGE_HEIGHT = 300
MIN_GOAL_HEIGHT = 0.25
MIN_GOAL_WIDTH = 0.125

def sanity_check(ret):
  center, x_width = ret
  ret = True
  ret &= (center[1] / float(IMAGE_HEIGHT)) > MIN_GOAL_HEIGHT
  ret &= (x_width / float(IMAGE_WIDTH)) > MIN_GOAL_WIDTH
  return ret
  
def filter_wall(img):
  img = np.array(img)
  mask = (img[:,:,0] > WHITE_THRESHOLD) & \
         (img[:,:,1] > WHITE_THRESHOLD) & \
         (img[:,:,2] > WHITE_THRESHOLD)
  wall_filter_mask = np.zeros(mask.shape)
  # loop through the image column by column, 
  # look for the first white pixel, and blank 
  # everything before that
  for j in range(mask.shape[1]):
    for i in range(mask.shape[0]):
      if not mask[i][j]:
        wall_filter_mask[i][j] = True
      else: break
  np.place(img[:,:,0], wall_filter_mask, 0)
  np.place(img[:,:,1], wall_filter_mask, 0)
  np.place(img[:,:,2], wall_filter_mask, 0)
  return Image.fromarray(img)
  
def get_goal_mask(orig_img):
  img = orig_img.filter(ImageFilter.GaussianBlur(BLUR_RADIUS))
  img = np.array(img.convert("HSV"))
  hue, sat = img[:, :, 0], img[:, :, 1]
  mask = (hue > HUE_LOWER_THRESHOLD)  \
       & (hue < HUE_HIGHER_THRESHOLD) \
       & (sat > SATURATION_THRESHOLD)
  # Image.fromarray((mask*255).astype(np.uint8)).show( )
  return mask

def detect_shape(c):
  shape = "unidentified"
  peri = cv2.arcLength(c, True)
  approx = cv2.approxPolyDP(c, 0.04 * peri, True)
  if len(approx) == 3:
    shape = "triangle"
  elif len(approx) == 4:
    (x, y, w, h) = cv2.boundingRect(approx)
    ar = w / float(h)
    shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
  elif len(approx) == 5:
    shape = "pentagon"
  else:
    shape = "circle"
  return shape

def polygon_area(x, y):
  first = np.dot(x, np.roll(y,1))
  second = np.dot(y, np.roll(x, 1))
  return 0.5 * np.abs(first - second)
    
def get_goal_params(orig_img):
  orig_img_arr = np.array(orig_img)
  # img = filter_wall(orig_img)
  img = get_goal_mask(orig_img)
  cnts = cv2.findContours(img.astype(np.uint8), 
                          cv2.RETR_EXTERNAL,
                          cv2.CHAIN_APPROX_SIMPLE)
  is_cv3 = cv2.__version__.startswith("3.")
  cnts = cnts[1] if is_cv3 else cnts[0]
  for c in cnts:
    M = cv2.moments(np.array(c))
    if M["m00"] != 0:
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
      shape = detect_shape(c)
      # draw the contour and center of the shape on the image
      if shape == "rectangle" or \
         shape == "square"    or \
         shape == "pentagon":
        points = np.array(c).reshape(-1, 2)
        x_distance = points[:, 0].max() - points[:, 0].min()
        ret = (cX, cY), x_distance
        if sanity_check(ret):
          return (cX, cY), x_distance
  return (0, 0), 0
  
if __name__ == "__main__":
  img = Image.open("/Users/KOCABEY/Desktop/NUC/team-1/opencv/goal_arena.jpg")
  print get_goal_params(img)
  # img = filter_wall(img)
  # u = get_goal_mask(img)
  # Image.fromarray((u*255).astype(np.uint8)).show()
