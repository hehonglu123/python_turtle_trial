import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError

bridge=CvBridge()
def callback(data):
	#triggered when data received
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")	#convert ros image message to opencv object
    except CvBridgeError as e:
        print(e)
    cv2.namedWindow("Image")
    if (not cv_image is None):
        cv2.imshow("Image",cv_image)
    if cv2.waitKey(50)==-1:
        cv2.destroyAllWindows()


if __name__ == '__main__': 
	#intialize rosnode and subscriber
    rospy.init_node('stream_node', anonymous=False)
    sub = rospy.Subscriber("image_raw",Image,callback, queue_size = 1)
    #run continuously
    rospy.spin()
        