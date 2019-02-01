import enum
import rospy
from time import time
from Queue import Queue
from time import sleep
from random import randint
from std_msgs.msg import String
from pid_controller.pid import PID

# Controller Settings
BALL_P, BALL_I, BALL_D = -30.0, 0.0, 0.0
GOAL_P, GOAL_I, GOAL_D = -30.0, 0.0, 0.0
RATE = 10
CENTERING_THRESHOLD = 0.0
# Ball Paramsss
BALL_MOVE_EPSILON = 10
# Explore Params
EXPLORE_ROTATE_EPSILON = 20 
EXPLORE_MOVE_EPSILON = 10
EXPLORE_MOVE_COUNT = 25
EXPLORE_ROTATE_COUNT = 15
EXTRA_ROTATE_COUNT_MAX = 10
# Safety Params
SAFETY_MOVE_BACK = 15
SAFETY_MOVE_FORWARD = 50
SAFETY_MOVE_BACK_WAIT_COUNT = 5
SAFETY_MOVE_FORWARD_WAIT_COUNT = 5
SAFETY_ROTATE_BACK_WAIT_COUNT = 5
# Goal Params
GOAL_MOVE_EPSILON = 10
# GOAL_DISTANCE_THRESHOLD = 300
GOAL_WIDTH_THRESHOLD = 0.8
GOAL_ROTATION_SLEEP = 2
GOAL_SERVO_SLEEP = 4
GOAL_BACK_SLEEP = 2
GOAL_TIME_THRESHOLD = 20

TIMEOUT_THRESHOLD = 30

# MAX_TOF_DIST = 8192
# MIN_TOF_DIST = 250

class State(enum.Enum):
  EXPLORE = 1
  SAFETY = 2
  BALL = 3
  GOAL = 4
  
class Brain(object):
  
  def __init__(self):
    rospy.init_node('brain')
    rospy.Subscriber("ball_info", String, self.ball_info_callback)
    rospy.Subscriber("goal_info", String, self.goal_info_callback)
    # rospy.Subscriber("tof_readings", String, self.tof_readings_callback)
    self.pub = rospy.Publisher('driver_commands', String, queue_size=10)
    self.ball_pid = PID(p=BALL_P, i=BALL_I, d=BALL_D)
    self.goal_pid = PID(p=GOAL_P, i=GOAL_I, d=GOAL_D)
    self.ball_center = 0
    self.ball_radius = 0
    self.goal_center = 0
    self.goal_size = 0
    self.state = State.EXPLORE
    self.plan = Queue()
    # TOF Readings
    # self.dist1 = MAX_TOF_DIST
    # self.dist2 = MAX_TOF_DIST
    # self.danger = 1
    self.time = time()
    self.index = 0
    self.last_commands = [i for i in range(TIMEOUT_THRESHOLD)]
  
  def start(self):
    rate = rospy.Rate(RATE)
    # Main Loop
    while not rospy.is_shutdown():
      # print "Distance Readings: ", self.dist1, self.dist2
      # If there is a very close wall
      # print self.last_commands
      if self.state != State.SAFETY:
        if len(set(self.last_commands)) == 1:
          self.state = State.SAFETY
          self.plan = Queue()
          print "Switched to Safety State"
          continue
        pass
      # --------------------------------------- #
      #               Safety State              #
      # --------------------------------------- #
      if self.state == State.SAFETY:
        if self.plan.qsize() == 0:
          self.plan.put(("move", -SAFETY_MOVE_BACK))
          for i in range(SAFETY_MOVE_BACK_WAIT_COUNT):
            self.plan.put("sleep")
          self.plan.put(("rotate", 180))
          for i in range(SAFETY_ROTATE_BACK_WAIT_COUNT):
            self.plan.put("sleep")
          self.plan.put(("move", SAFETY_MOVE_FORWARD))
          for i in range(SAFETY_MOVE_FORWARD_WAIT_COUNT):
            self.plan.put("sleep")
          self.plan.put("done")
        else:
          current = self.plan.get()
          if current == "done":
            self.state = State.EXPLORE
            self.plan = Queue()
            print "Switched to Explore State"
          elif current == "sleep":
            pass
          else:
            self.pub.publish("{} {}".format(current[0], current[1]))
            print "Safety: {} {}".format(current[0], current[1])
            self.last_commands[self.index] = "{}".format(current[0])
            self.index += 1
            self.index %= TIMEOUT_THRESHOLD
      # --------------------------------------- #
      #              Explore State              #
      # --------------------------------------- #
      elif self.state == State.EXPLORE:
        # If there is a goal in vision
        # and enough time has passed
        if self.goal_center > 1e-3 and time() - self.time > GOAL_TIME_THRESHOLD:
          self.state = State.GOAL
          print "Switched to Goal State"
          continue
        if self.ball_center > 1e-3:
          self.state = State.BALL
          print "Switched to Ball State"
          continue
        if self.plan.qsize() == 0:
          extra_rotate_count = randint(0, EXTRA_ROTATE_COUNT_MAX)
          for i in range(EXPLORE_MOVE_COUNT):
            self.plan.put(("move", EXPLORE_MOVE_EPSILON))
          for i in range(EXPLORE_ROTATE_COUNT + extra_rotate_count):
            self.plan.put(("rotate", EXPLORE_ROTATE_EPSILON))
        else:
          current = self.plan.get()
          self.pub.publish("{} {}".format(current[0], current[1]))
          print "Explore: {} {}".format(current[0], current[1])
          self.last_commands[self.index] = "{}".format(current[0])
          self.index += 1
          self.index %= TIMEOUT_THRESHOLD
      # --------------------------------------- #
      #               Ball State                #
      # --------------------------------------- #
      elif self.state == State.BALL:
        # If there is no ball in vision
        if self.ball_center <= 1e-3:
          self.state = State.EXPLORE
          self.plan = Queue()
          print "Switched to Explore State"
          continue
        diff = self.ball_center - 0.5
        if abs(diff) < CENTERING_THRESHOLD:
          self.pub.publish("move " + str(BALL_MOVE_EPSILON))
          print "Ball: move {}".format(BALL_MOVE_EPSILON)
          self.last_commands[self.index] = "move"
          self.index += 1
          self.index %= TIMEOUT_THRESHOLD
        else:
          angle = self.ball_pid(feedback=diff)
          self.pub.publish("rotate " + str(angle))
          print "Ball: rotate {}".format(angle)
          self.last_commands[self.index] = "rotate"
          self.index += 1
          self.index %= TIMEOUT_THRESHOLD
      # --------------------------------------- #
      #               Goal State                #
      # --------------------------------------- #
      elif self.state == State.GOAL:
        # If there is no goal in vision
        if self.goal_center <= 1e-3:
          self.state =  State.EXPLORE
          self.plan = Queue()
          print "Switched to Explore State"
          continue
        
        if self.goal_size > GOAL_WIDTH_THRESHOLD:
          self.pub.publish("rotate 180")
          sleep(GOAL_ROTATION_SLEEP)
          self.pub.publish("release balls")
          sleep(GOAL_SERVO_SLEEP)
          self.pub.publish("move -5")
          sleep(GOAL_BACK_SLEEP)
          self.state = State.EXPLORE
          self.plan = Queue()
          print "Switched to Explore State"
          continue
           
        diff = self.goal_center - 0.5
        if abs(diff) < CENTERING_THRESHOLD:
          self.pub.publish("move " + str(GOAL_MOVE_EPSILON))
          print "Goal: move {}".format(GOAL_MOVE_EPSILON)
          self.last_commands[self.index] = "move"
          self.index += 1
          self.index %= TIMEOUT_THRESHOLD
        else:
          angle = self.goal_pid(feedback=diff)
          self.pub.publish("rotate " + str(angle))
          print "Goal: rotate {}".format(angle)
          print self.goal_center
          self.last_commands[self.index] = "rotate"
          self.index += 1
          self.index %= TIMEOUT_THRESHOLD
          
      rate.sleep()
      
  def ball_info_callback(self, data):
    center, radius = data.data.split()
    self.ball_center = float(center)
    self.ball_radius = float(radius)
    print self.ball_center
  
  def goal_info_callback(self, data):
    center, size = data.data.split()
    self.goal_center = float(center)
    self.goal_size = float(size)
  # 
  # def tof_readings_callback(self, data):
  #   dist1, dist2 = data.data.split()
  #   self.dist1 = float(dist1)
  #   self.dist2 = float(dist2)
  
if __name__ == "__main__":
  brain = Brain()
  try:
    brain.start()
  except rospy.ROSInterruptException:
    pass
