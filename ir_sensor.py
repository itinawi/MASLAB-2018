from tamproxy import Sketch, SyncedSketch, Timer

from tamproxy.devices import AnalogInput

# Reads the analog voltage at one of the analog pins

class AnalogRead(SyncedSketch):
    adc_pin = 23

    def setup(self):
        self.testpin = AnalogInput(self.tamp, self.adc_pin)
        self.timer = Timer()

    def loop(self):
        if self.timer.millis() > 100:
            self.timer.reset()
            print self.testpin.val
            volts= ((self.testpin.val)*0.0001)-1
            #print(volts) #at 5 volts this seems to be a good distance from wall?
            distance= float(13.0/(volts))
            print distance
            if distance>0 and distance< 2.6:
                print("STOP!! then turn")
            elif distance >2.6:
                print("MOVE AHEAD")
            elif distance <0:
                print("MOVE AHEAD")

if __name__ == "__main__":
    sketch = AnalogRead(1, -0.00001, 100)
    sketch.run()
