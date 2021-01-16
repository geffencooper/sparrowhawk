#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import cv2
import numpy as np
from PIL import Image
import urllib.request
import imutils
import argparse

#ap = argparse.ArgumentParser()
#ap.add_argument("-p", "--prototxt", required=False,
#    help="path to Caffe 'deploy' prototxt file")
#ap.add_argument("-m", "--model", required=False,
#    help="path to Caffe pre-trained model")
#ap.add_argument("-c", "--confidence", type=float, default=0.2,
#    help="minimum probability to filter weak detections")
#args = vars(ap.parse_args())

CLASSES = ["person"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# load our serialized model from disk
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe("/home/geffen/sparrowhawk/home_base_ws/src/home_utils/scripts/MobileNetSSD_deploy.prototxt.txt", "/home/geffen/sparrowhawk/home_base_ws/src/home_utils/scripts/MobileNetSSD_deploy.caffemodel")


def tracker():
    pub = rospy.Publisher('tracker', Point, queue_size=10)
    rospy.init_node('tracker', anonymous=True)
    rate = rospy.Rate(10)
    vcap = cv2.VideoCapture("http://192.168.0.129:8080/stream/video.mjpeg")
    width = vcap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = vcap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    while not rospy.is_shutdown():
        ret, frame = vcap.read()

        frame = cv2.flip(frame, 0)

          # grab the frame dimensions and convert it to a blob
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(
            frame, (300, 300)), 0.007843, (300, 300), 127.5)

        # pass the blob through the network and obtain the detections and
        # predictions
        net.setInput(blob)
        detections = net.forward()

        # loop over the detections
        for i in np.arange(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with
            # the prediction
            confidence = detections[0, 0, i, 2]

            # filter out weak detections by ensuring the `confidence` is
            # greater than the minimum confidence
            p = Point()
            p.z = 0
            if confidence > 0.95:
                # extract the index of the class label from the
                # `detections`, then compute the (x, y)-coordinates of
                # the bounding box for the object
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # draw the prediction on the frame
                label = "{}: {:.2f}%".format(CLASSES[0],
                    confidence * 100)
                cv2.rectangle(frame, (startX, startY), (endX, endY),
                    COLORS[0], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[0], 2)
                
                centerX = (int)((endX+startX)/2)
                centerY = (int)(abs(endY+startY)/2)
                #p.x = (centerX - width/2)/width
                p.x = 90 if (centerX > width/2) else -90
                p.y = centerY - height/2
                p.z = abs(endX-startX) * abs(endY-startY)
                #print(p.z)
                # print("coords:")
                # print(startX, startY,  endX, endY)
                # print("center")
                # print(centerX, centerY)
                cv2.putText(frame, "TL", (startX, startY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[0], 2)
                cv2.putText(frame, "center", (centerX, centerY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[0], 2)
                cv2.putText(frame, "TR", (endX, endY),cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[0], 2)

                pub.publish(p)
          # output current frame and masked frame
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # cv2.waitKey(0)
        # lower_range = np.array([0,100,100])
        # upper_range = np.array([200,255,255])

    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tracker()
    except rospy.ROSInterruptException:
        pass
