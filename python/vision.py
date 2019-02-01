import cv2
import rospy
import numpy as np
from time import sleep
from PIL import Image
from std_msgs.msg import String
from ball import get_closest_ball
from goal import get_goal_params

DEFAULT_IMAGE_SIZE = (400, 300)

class Vision(object):
  
  def __init__(self):
    rospy.init_node('vision')
    self.cap = cv2.VideoCapture(0)
    self.ball_pub = rospy.Publisher('ball_info', String, queue_size=10)
    self.goal_pub = rospy.Publisher('goal_info', String, queue_size=10)
  
  def start(self):
    while not rospy.is_shutdown():
      ret, frame = self.cap.read()
      ball_center, ball_radius = get_ball_info(frame)
      goal_center, goal_size = get_goal_info(frame)
      self.ball_pub.publish("{} {}".format(ball_center, ball_radius))
      self.goal_pub.publish("{} {}".format(goal_center, goal_size))
      print "Ball Center: ", ball_center
      print "Goal Center: ", goal_center
      sleep(0.1)
      
def get_ball_info(img):
  img = Image.fromarray(img[:, :, ::-1])
  img = img.resize(DEFAULT_IMAGE_SIZE)
  ball_info = get_closest_ball(img)
  center_x = ball_info[0][0] / float(DEFAULT_IMAGE_SIZE[0])
  radius = ball_info[1] / float(DEFAULT_IMAGE_SIZE[0])
  return center_x, radius

def get_goal_info(img):
  img = Image.fromarray(img[:, :, ::-1])
  img = img.resize(DEFAULT_IMAGE_SIZE)
  goal_info = get_goal_params(img)
  center_x = goal_info[0][0] / float(DEFAULT_IMAGE_SIZE[0])
  size = goal_info[1] / float(DEFAULT_IMAGE_SIZE[0])
  return center_x, size
  
if __name__ == "__main__":
  vision = Vision()
  try:
    vision.start()
  except rospy.ROSInterruptException:
    pass
