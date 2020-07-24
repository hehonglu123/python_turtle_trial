#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Webcam_impl(): 
    def __init__(self): 
        #initializa camera parameters
        self.camera = cv2.VideoCapture(0)
        self.camera.set(3,320)
        self.camera.set(4,240)

    def CaptureFrame(self): 
        rval,img_data = self.camera.read()
        if rval:
            return img_data 
        else:
            print("error")
 
 
if __name__ == '__main__': 
    #initialize camera object, rosnode and camera
    webcam=Webcam_impl() 
    pub = rospy.Publisher('image_raw', Image, queue_size=1) 
    rospy.init_node('webcam', anonymous=False)
    bridge=CvBridge()
    print("webcam node started")
    while not rospy.is_shutdown(): 
        #continuously capturing and publishing image messages
        frame=webcam.CaptureFrame()
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8")) 
