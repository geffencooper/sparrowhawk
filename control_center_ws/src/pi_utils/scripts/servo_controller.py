#!/usr/bin/env python3

# servo control class
import pigpio
import time

pi = pigpio.pi()

class ServoController:
  def __init__(self, pwm_pin, cal_file = "", minimum=0, center=0, maximum=0):
     
    self.pwm_pin = pwm_pin 

    # initialize servo minimum, center, and maximum positions based on duty cycle
    self.minimum = minimum
    self.center = center
    self.maximum = maximum
    
    # initialize above values based on calibration file
    if cal_file != "":
        f = open(cal_file, "r")
        self.minimum = float(f.readline())
        self.center = float(f.readline())
        self.maximum = float(f.readline())

    # set the current duty cyle to the center which is "rest state"
    self.current_dc = self.center
    self.raw(self.center)

  # directly set the duty cycle with this function
  def raw(self, dc):
    if self.current_dc != dc: # only update dc if different than current
        print("new dc: ",dc)
        self.current_dc = dc
        pi.set_servo_pulsewidth(self.pwm_pin, dc)

  def forward(self, speed): # input is relative number 0-100 --> convert to servo parameter
    duty_cycle = (float(speed)*(self.maximum - self.center))/100.0 + self.center
    #print(duty_cycle)
    self.raw(duty_cycle)
  
  def backward(self, speed): # input is relative number 0-100 --> convert to servo parameter
    duty_cycle = self.center - (float(speed)*(self.center - self.minimum))/100.0
    #print(duty_cycle)
    self.raw(duty_cycle)

  def turn(self, position): # number from -100 to 100, convert to servo parameter 
    if position == 0:
        self.raw(self.center)

    if position < 0:
        # turn left
        position = position*-1
        self.backward(position)
    elif position > 0:
        # turn right
        self.forward(position)

  def stop(self):
    self.raw(self.center)

  def pause(self):
    self.raw(self.center)
  
  
  def calibrate(self):
    print("\n\n------------------manual servo calibration-------------")
    print("\ntype in duty cycle values to determine min, max, center.")
    print("\nwhen satisfied type -1")
    calName = input("calibration name: ")

    self.raw(1500)
    time.sleep(5.0)
    print("\nMINIMUM:")
    dc = (float)(input("DC(~500-1500): "))
    while(dc > 0):
        self.raw(dc)
        time.sleep(1)
        self.minimum = dc
        dc = (float)(input("DC: "))
    print("MINIMUM = ", self.minimum)
    
    print("\nCENTER:")
    dc = (float)(input("DC(~1500):"))
    
    while(dc > 0):
      self.raw(dc)
      time.sleep(1)
      self.center = dc
      dc = (float)(input("DC:"))
    print("CENTER = ", self.center)

    print("\nMAXIMUM:")
    dc = (float)(input("DC(~1500-2500): "))
    
    while(dc > 0):
      self.raw(dc)
      time.sleep(1)
      self.maximum = dc
      dc = (float)(input("DC:"))
    print("MAXIMUM = ", self.maximum)

    print("\n\n CALIBRATION COMPLETE")
    print("writing results to: ",calName,"_cal.txt")
    fileName = calName + "_cal.txt"
    f = open(fileName, "a")
    f.write(str(self.minimum)+"\n")
    f.write(str(self.center)+"\n")
    f.write(str(self.maximum)+"\n")
    

'''if __name__ == '__main__':
    sSteer = servoController(13, "/home/geffen-pi/test_ws/src/testPkg/scripts/steeringCal.txt")
    sDrive = servoController(19, "/home/geffen-pi/test_ws/src/testPkg/scripts/driveCal.txt")
    print("start")
    sSteer.turn(80)
    #sSteer.raw(1768.0)
    time.sleep(1)
    #sSteer.turn(-100)
    time.sleep(1)
    sSteer.pause()
   # c2.raw(5)
   # c.calibrate()
   # c2.calibrate()
    time.sleep(1)'''