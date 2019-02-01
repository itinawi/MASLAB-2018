# import the necessary packages
import cv2
import numpy as np
from PIL import Image
import sys
from get_image import *
import datetime
import time

# if an image path is defined, use that path. Otherwise, take an image with the webcam. If that fails, default to "/home/maslab/opencv/red_image.png"
if len(sys.argv) > 1:
    filepath = sys.argv[1]
else:
    ts = time.time()
    t = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
    filepath = "/home/maslab/opencv/images/image_" + t + ".png"
    write_image(filepath)
