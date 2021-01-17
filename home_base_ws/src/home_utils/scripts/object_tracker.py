#!/usr/bin/env python3

# Note: most of the object detection code is taken from an online source
# I integrated that code into a ROS node which publishes bounding box info

import rospy
from geometry_msgs.msg import Point
import cv2
import numpy as np
from PIL import Image
import urllib.request
import imutils
import argparse

# for now let's detect people
CLASSES = ["person"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe("/home/geffen/sparrowhawk/home_base_ws/src/home_utils/scripts/MobileNetSSD_deploy.prototxt.txt", "/home/geffen/sparrowhawk/home_base_ws/src/home_utils/scripts/MobileNetSSD_deploy.caffemodel")

# ROS node function
def tracker():
    # init the publisher
    pub = rospy.Publisher('tracker', Point, queue_size=10)
    rospy.init_node('tracker', anonymous=True)
    rate = rospy.Rate(10)
    p = Point()
    last_z = 0
    
    # start capturing video
    vcap = cv2.VideoCapture("http://192.168.0.129:8080/stream/video.mjpeg")
    width = vcap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = vcap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    
    # start publishing
    while not rospy.is_shutdown():
        vcap = cv2.VideoCapture("http://192.168.0.129:8080/stream/video.mjpeg")
        ret, frame = vcap.read()

        # make sure to get a valid frame
        if ret == True:
            frame = cv2.flip(frame, 0)

            # grab the frame dimensions and convert it to a blob
            (h, w) = frame.shape[:2]
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

            # pass the blob through the network and obtain the detections and predictions
            net.setInput(blob)
            detections = net.forward()

            # loop over the detections
            for i in np.arange(0, detections.shape[2]):
                # extract the confidence (i.e., probability) associated with the prediction
                confidence = detections[0, 0, i, 2]

                # filter out weak detections by ensuring the `confidence` is greater than the minimum confidence
                if confidence > 0.95:
                    # extract index of class label from `detections`, compute (x, y)-coordinates of bounding box for object
                    idx = int(detections[0, 0, i, 1])
                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")

                    # draw the prediction on the frame
                    label = "{}: {:.2f}%".format(CLASSES[0],confidence * 100)
                    cv2.rectangle(frame, (startX, startY), (endX, endY), COLORS[0], 2)
                    y = startY - 15 if startY - 15 > 15 else startY + 15
                    cv2.putText(frame, label, (startX, y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[0], 2)
                    
                    centerX = (int)((endX+startX)/2)
                    centerY = (int)(abs(endY+startY)/2)
                    
                    cv2.putText(frame, "TL", (startX, startY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[0], 2)
                    cv2.putText(frame, "center", (centerX, centerY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[0], 2)
                    cv2.putText(frame, "TR", (endX, endY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[0], 2)
                    
                    # ROS geometry msg, x,y = dist from center of bounding box, z = area of bounding box
                    p.x = -((centerX - width/2)/width)*2.5
                    p.y = centerY - height/2
                    p.z = abs(endX-startX) * abs(endY-startY)
             
            # if go out of frame then "reverse" 
            # if (last_z == p.z) or (last_z*-1 == p.z):
            #     p.z *= -1       
            pub.publish(p)
            # last_z = p.z
            
            # output current frame and masked frame
            cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tracker()
    except rospy.ROSInterruptException:
        pass
