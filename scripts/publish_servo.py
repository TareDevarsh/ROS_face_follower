#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt16
import cv2
import numpy as np

x = 0
def talker(x):
    pub = rospy.Publisher('servo', UInt16, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    if not rospy.is_shutdown():
        print(x)
        hello_str = x
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def face_reco():
    face_cascade = cv2.CascadeClassifier('/home/dev/catkin_ws/src/opencv_follow/scripts/haarcascade_frontalface_alt.xml')
    cap = cv2.VideoCapture(0)
    red = [0,0,255]
    while(True):
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        if (x >70):
            talker(int(x))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        face_detect = face_cascade.detectMultiScale(gray, scaleFactor = 1.1, minNeighbors = 5)
        for (x, y , w , h) in face_detect:
            if ((x+w) * (y+h) > 70000):
                cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,0),3)    
                center  = x , y   
                cv2.circle(frame, center, 30,(255,0,0), thickness=2)
                x = float(x) / 500
                x = x * 90 + 70
            else:
                continue
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == '__main__':
    try:
       face_reco()
        
    except rospy.ROSInterruptException:
        pass