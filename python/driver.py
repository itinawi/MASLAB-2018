import rospy
import ctypes
import numpy as np
from time import sleep
from Queue import Queue
from threading import Thread
from std_msgs.msg import String
from pid_controller.pid import PID

from tamproxy import Sketch, SyncedSketch, Timer
from tamproxy.devices import Motor, Encoder, TimeOfFlight, Servo

# Pin Numbers
MOTOR1_PWM, MOTOR1_DIR = 9, 11
MOTOR2_PWM, MOTOR2_DIR = 10, 12
ENCODER1 = 36, 35
ENCODER2 = 38, 37
# TOF1, TOF2 = 22, 4
SERVO = 6

# Controller Parameters
MAX_SPEED = 80
CONTROLLER_THRESHOLD = 10
P, I, D = 0.1, 0.0, 0.0
MOTOR_THRESHOLD = 50
TURN_COEFFICIENT = 1.01 # should be tuned based on surface

# Miscellaneous
SLEEP_TIME = 0.05
MOTOR1_DIRECTION = 1.0
MOTOR2_DIRECTION = -1.0
FULL_TICKS = 3200
SERVO_CLOSE = 180
SERVO_OPEN = 0
SERVO_WAIT = 5

# Measurements in Centimeters
WHEEL_DISTANCE = 31.115
WHEEL_THICKNESS = 2.032
WHEEL_RADIUS = 4.921
ROBOT_RADIUS = WHEEL_DISTANCE / 2

class Driver(Sketch):
  
  def setup(self):
    # ROS Setup
    rospy.init_node('driver')
    rospy.Subscriber("driver_commands", String, callback)
    # self.tof_pub = rospy.Publisher('tof_readings', String, queue_size=10)
    # Device Setup
    self.motor1 = Motor(self.tamp, MOTOR1_DIR, MOTOR1_PWM)
    self.motor2 = Motor(self.tamp, MOTOR2_DIR, MOTOR2_PWM)
    self.encoder1 = Encoder(self.tamp, *ENCODER1, continuous=True)
    self.encoder2 = Encoder(self.tamp, *ENCODER2, continuous=True)
    # self.encoder1_value = 0
    # self.encoder2_value = 0
    # TimeOfFlight Sensors
    # self.tof1 = TimeOfFlight(self.tamp, TOF1, 1, continuous=False)
    # self.tof2 = TimeOfFlight(self.tamp, TOF2, 2, continuous=False)
    # self.tof1.enable()
    # self.tof2.enable()
    # self.tof1_value = 0
    # self.tof2_value = 0
    # Servo for the Release
    self.servo = Servo(self.tamp, SERVO)
    self.servo.write(SERVO_CLOSE)
    # Controller Params
    self.pid = PID(p=P, i=I, d=D)
    self.target1 = 0
    self.target2 = 0 
    self.servo_commands = Queue()
  
  def rotate(self, angle):
    target_rotation = (angle / 360.0) * ROBOT_RADIUS / WHEEL_RADIUS
    target_ticks = FULL_TICKS * TURN_COEFFICIENT * target_rotation
    self._rotate(target_ticks)
  
  def _rotate(self, ticks):
    self.target1 = self.encoder1.val + MOTOR1_DIRECTION * ticks
    self.target2 = self.encoder2.val + MOTOR2_DIRECTION * ticks
  
  def move(self, distance):
    target_rotation = distance / (2 * np.pi * WHEEL_RADIUS)
    target_ticks = target_rotation * FULL_TICKS
    self.target1 = self.encoder1.val - MOTOR1_DIRECTION * target_ticks
    self.target2 = self.encoder2.val + MOTOR2_DIRECTION * target_ticks
    
  def release(self):
    self.servo_commands.put(SERVO_OPEN)
    sleep(SERVO_WAIT)
    self.servo_commands.put(SERVO_CLOSE)
    
  def loop(self):
    sleep(SLEEP_TIME)
    # Send Sensor Requests
    # self.tamp.send_request(self.encoder1.id, 
    #                        self.encoder1.READ_CODE, 
    #                        self._handle_encoder1_update, 
    #                        continuous=False,
    #                        weight=1)
    # self.tamp.send_request(self.encoder2.id, 
    #                        self.encoder2.READ_CODE, 
    #                        self._handle_encoder2_update, 
    #                        continuous=False, 
    #                        weight=1)
    # self.tamp.send_request(self.tof1.id, 
    #                        self.tof1.READ_CODE,
    #                        self._handle_tof1_update,
    #                        continuous=False, 
    #                        weight=1)
    # self.tamp.send_request(self.tof2.id, 
    #                        self.tof2.READ_CODE, 
    #                        self._handle_tof2_update, 
    #                        continuous=False, 
    #                        weight=1)
    if self.servo_commands.qsize() != 0:
      self.servo.write(self.servo_commands.get())
    # Main Control Logic
    diff1 = self.target1 - self.encoder1.val
    diff2 = self.target2 - self.encoder2.val
    if abs(diff1) < CONTROLLER_THRESHOLD: diff1 = 0
    if abs(diff2) < CONTROLLER_THRESHOLD: diff2 = 0
    write(self.motor1, MOTOR1_DIRECTION, self.pid(feedback=diff1))
    write(self.motor2, MOTOR2_DIRECTION, self.pid(feedback=diff2))
    # Publish ToF Readings
    # self.tof_pub.publish("{} {}".format(self.tof1_value, self.tof2_value))
    
  # def _handle_encoder1_update(self, request, response):
  #   new_val = ((ord(response[0]) << 24) |
  #              (ord(response[1]) << 16) |
  #              (ord(response[2]) << 8)  |
  #              (ord(response[3])))
  #   self.encoder1_value += ctypes.c_int32(new_val - self.encoder1_value).value
  # 
  # def _handle_encoder2_update(self, request, response):
  #   new_val = ((ord(response[0]) << 24) |
  #              (ord(response[1]) << 16) |
  #              (ord(response[2]) << 8)  |
  #              (ord(response[3])))
  #   self.encoder2_value += ctypes.c_int32(new_val - self.encoder2_value).value
  
  # def _handle_tof1_update(self, request, response):
  #   self.tof1_value = (ord(response[0]) << 8) + ord(response[1])
  #   print "ToF 1: ", self.tof1_value
  # 
  # def _handle_tof2_update(self, request, response):
  #   self.tof2_value = (ord(response[0]) << 8) + ord(response[1])
  #   print "ToF 2: ", self.tof2_value
  # 
def write(motor, direction, speed):
  speed = min(MAX_SPEED, speed)
  speed = max(-MAX_SPEED, speed)
  new_speed = speed * (255.0 - MOTOR_THRESHOLD) / 255.0
  if speed > 0: new_speed += MOTOR_THRESHOLD 
  if speed < 0: new_speed -= MOTOR_THRESHOLD 
  new_speed *= direction
  motor.write(new_speed > 0, abs(new_speed))
  
def square():
  sleep(3)
  while True:
    sketch.move(30)
    sleep(3)
    sketch.rotate(90)
    sleep(3)

def release():
  sleep(3)
  sketch.release()

def callback(data):
  command, amount = data.data.split()
  if command == "rotate":
    sketch.rotate(float(amount))
  if command == "move":
    sketch.move(float(amount))
  if command == "release":
    sketch.release()
    
if __name__ == "__main__":
  sketch = Driver()
  # Thread(target=square).start()
  sketch.run()
