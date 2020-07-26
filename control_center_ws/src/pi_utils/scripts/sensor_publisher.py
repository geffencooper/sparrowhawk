#!/usr/bin/env python3

import serial
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32


def data_getter():
     # a node to publish data from arduino
    rospy.init_node('sensor_publisher', anonymous=True)
    
    # publishes data to two topics, general sensor data and emergency
    data_pub = rospy.Publisher('sensor_data', Float32, queue_size = 10)
    emgncy_pub = rospy.Publisher('emergency', String, queue_size = 10)
    
    rate = rospy.Rate(10)
    
    # connect to arduino through USB seria;
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    while not rospy.is_shutdown():
        # read data until newline, convert to ascii, and remove trailing spaces
        data = ser.readline().decode('ascii').rstrip()
        print(data)
        if data != '':
            if data == 'initialized': # only continue if sensors initialized correctly
                ser.write(str.encode('startmapping\n'))
                global mapping
                mapping = True
            elif data == 'HC': # ultrasonic sensor data next
                data = ser.readline().decode('ascii').rstrip()
                if float(data) < 20:
                    if mapping == True:
                        emgncy_pub.publish("STOP")
                        print("Stopping now-----------------------------")
                        ser.write(str.encode('stopmapping\n'))
                        mapping = False
            elif data == 'TOF':
                data = ser.readline().decode('ascii').rstrip()
                print(data)
                if mapping == False:
                    mapping = True
                    ser.write(str.encode('startmapping\n'))
                data_pub.publish(data)


if __name__ == '__main__':
    try:
        mapping = False
        data_getter()
    except rospy.ROSInterruptexception:
        pass