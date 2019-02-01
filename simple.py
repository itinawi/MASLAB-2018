from time import sleep
from tamproxy import Sketch
from tamproxy.devices import Motor, AnalogInput, TimeOfFlight

class Simple(Sketch):
  
  def setup(self):
    
    self.motor = Motor(self.tamp, 11, 9)
    self.tof1 = TimeOfFlight(self.tamp, 20, 1)
    self.tof2 = TimeOfFlight(self.tamp, 22, 2)
    self.tof1.enable()
    self.tof2.enable()
    
  def loop(self):
    sleep(0.01)
    print self.tof1.dist, self.tof2.dist
    self.motor.write(1, 50)


if __name__ == "__main__":
  sketch = Simple()
  sketch.run()
