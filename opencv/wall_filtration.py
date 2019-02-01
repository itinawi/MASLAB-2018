import cv2
import numpy as np
from PIL import Image, ImageFilter
import math

WALL_THRESHOLD = 160

def averaged_wall_filter(img):
	mask = (img[:,:,2] > WALL_THRESHOLD) & (img[:,:,1] > WALL_THRESHOLD) & (img[:,:,0] > WALL_THRESHOLD)
	wall_filter_mask = np.zeros(mask.shape)

	# loop through the image column by column, look for the first white
	# pixel, and blank everything before that
	break_rows = np.zeros(mask.shape[1])  # array to keep track of breakpoint of each column
	for j in range(mask.shape[1]): # cols
	    for i in range(mask.shape[0]): # rows
	        if not mask[i][j]:
	            wall_filter_mask[i][j] = True
	        else:
	            break_rows[j] = i
	            break
	filter_y = int(math.floor(np.median(break_rows)))

	bad_cols = []
	for i in range(len(break_rows)):
	    elem = break_rows[i]
	    if elem > mask.shape[0]/2 or elem == 0:
	        break_rows[i] = filter_y
	        bad_cols.append(i)

	for j in bad_cols:
	    for i in range(1, filter_y):
	        wall_filter_mask[i][j] = True
	    for i in range(filter_y, mask.shape[0]):
	        wall_filter_mask[i][j] = False

	# comment back if you want to see pre-averaged image.
	# np.place(img[:,:,0], wall_filter_mask, 0)
	# np.place(img[:,:,1], wall_filter_mask, 0)
	# np.place(img[:,:,2], wall_filter_mask, 0)

	# comment back if you want to display
	# Image.fromarray(img.astype(np.uint8))

	for j in range(mask.shape[1]):
	    for i in range(filter_y):
	        wall_filter_mask[i][j] = True
	np.place(img[:,:,0], wall_filter_mask, 0)
	np.place(img[:,:,1], wall_filter_mask, 0)
	np.place(img[:,:,2], wall_filter_mask, 0)

	# comment back to display
	# Image.fromarray(img.astype(np.uint8))
	return img


def relaxed_wall_filter(img):
	mask = (img[:,:,2] > WALL_THRESHOLD) & (img[:,:,1] > WALL_THRESHOLD) & (img[:,:,0] > WALL_THRESHOLD)
	wall_filter_mask = np.zeros(mask.shape)

	# loop through the image column by column, look for the first white
	# pixel, and blank everything before that
	for j in range(mask.shape[1]): # cols
	    for i in range(mask.shape[0]): # rows
	        if not mask[i][j]:
	            wall_filter_mask[i][j] = True
	        else:
	            break

	np.place(img[:,:,0], wall_filter_mask, 100)
	np.place(img[:,:,1], wall_filter_mask, 100)
	np.place(img[:,:,2], wall_filter_mask, 100)

	# uncomment if you want to view the image
	# Image.fromarray(img.astype(np.uint8))
	return img



