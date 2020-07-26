#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Point
from servo_controller import ServoController

# create servo controller objects for driving and steering servos
steer_servo = ServoController(13, "/home/geffen-pi/sparrowhawk/control_center_ws/src/pi_utils/servo_calibration_files/steering_cal.txt")
drive_servo = ServoController(19, "/home/geffen-pi/sparrowhawk/control_center_ws/src/pi_utils/servo_calibration_files/drive_cal.txt")

# global stop variable
stop = False

# emergency callback
def emergency_callback(msg):
    if msg.data == "STOP":
        print("===============STOP================")
        drive_servo.backward(100)
        time.sleep(1500)
        drive_servo.pause()
        stop = True
    elif msg.data == "RESUME":
        print("==============RESUME==============")
        stop = False

def key_state_callback(msg):
    print(msg.data)
    if(msg.data == "R"):
        steer_servo.turn(100)
    elif(msg.data == "L"):
        steer_servo.turn(-100)
    # CAN ONLY GO FORWARD IF STOP = FALSE
    elif(not stop and msg.data == "U"):
        drive_servo.forward(100)
    elif(msg.data == "D"):
        drive_servo.backward(100)
    elif(not stop and msg.data == "UR"):
        drive_servo.forward(100)
        steer_servo.turn(100)
    elif(not stop and msg.data == "UL"):
        drive_servo.forward(100)
        steer_servo.turn(-100)
    elif(msg.data == "DR"):
        drive_servo.backward(100)
        steer_servo.turn(100)
    elif(msg.data == "DL"):
        drive_servo.backward(100)
        steer_servo.turn(-100)
    elif(msg.data == "PT"):
        steer_servo.pause()
    elif(msg.data == "PD"):
        drive_servo.pause()


if __name__ == '__main__':
    rospy.init_node('manual_controller') # the manual controller node subscribes to the key_publisher/emergency topics

    rospy.Subscriber('keys_pressed', String, key_state_callback)
    rospy.Subscriber('emergency', String, emergency_callback)
    rospy.spin()
