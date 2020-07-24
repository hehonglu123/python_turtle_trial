import turtle
import cv2
import time
import numpy as np
import rospy		#import ROS library
from python_turtle.msg import turtle_msg
from python_turtle.srv import setcolor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#display setup
screen = turtle.Screen()
screen.bgcolor("lightblue")
t1=turtle.Turtle()
t1.shape("turtle")
#ROS initialization
rospy.init_node('client_color', anonymous=False)
bridge=CvBridge()
rospy.wait_for_service('setcolor')
set_color=rospy.ServiceProxy('setcolor',setcolor)
ret=set_color("red")
#vel publisher
pub=rospy.Publisher('drive',Twist,queue_size=1)
turtle_obj=turtle_msg()

def callback(data):
	global turtle_obj
	turtle_obj=data

def image_callback(data):
	#triggered when data received
	try:
		image = bridge.imgmsg_to_cv2(data, "bgr8")	#convert ros image message to opencv object
	except CvBridgeError as e:
		print(e)
	cv2.namedWindow("Image")
	if (not image is None):
		cv2.imshow("Image",image)
	if cv2.waitKey(50)==-1:
		cv2.destroyAllWindows()


	image_size=data.width*data.height
	image_dimension=np.array([data.height,data.width])

	msg=Twist()

	# 1) filter on RED component
	image_red = cv2.inRange(image, np.array([5,5,200]),np.array([200,200,255]))
	#run color connected components to filter the counts and centroid
	retval, labels, stats, centroids = cv2.connectedComponentsWithStats(image_red)
	idx=np.where(np.logical_and(stats[:,4]>=0.01*image_size, stats[:,4]<=0.5*image_size))[0]    #threshold the components to find the best one
	for i in idx:
		if np.linalg.norm(centroids[i]-image_dimension/2.)<50:  #threshold again, only for ones near the center
			print("red detected")
			msg.linear.x=10
			pub.publish(msg)
			return

	# 2) filter on GREEN component
	image_green = cv2.inRange(image, np.array([5,200,5]),np.array([200,255,200]))
	#run color connected components to filter the counts and centroid
	retval, labels, stats, centroids = cv2.connectedComponentsWithStats(image_green)
	idx=np.where(np.logical_and(stats[:,4]>=0.01*image_size, stats[:,4]<=0.5*image_size))[0]    #threshold the components to find the best one
	for i in idx:
		if np.linalg.norm(centroids[i]-image_dimension/2.)<50:  #threshold again, only for ones near the center
			print("green detected")
			msg.angular.z=-10
			pub.publish(msg)
			return

	# 3) filter on BLUE component
	image_blue = cv2.inRange(image, np.array([200,5,5]),np.array([255,200,200]))
	#run color connected components to filter the counts and centroid
	retval, labels, stats, centroids = cv2.connectedComponentsWithStats(image_blue)
	idx=np.where(np.logical_and(stats[:,4]>=0.01*image_size, stats[:,4]<=0.5*image_size))[0]    #threshold the components to find the best one
	for i in idx:
		if np.linalg.norm(centroids[i]-image_dimension/2.)<50:  #threshold again, only for ones near the center
			print("blue detected")
			msg.angular.z=10
			pub.publish(msg)
			return

def updatepose():                    #set a new pose for turtlebot
	global turtle_obj
	if turtle_obj.color=="None":
		t1.penup()
	else:
		t1.pencolor(turtle_obj.color)

	t1.setpos(turtle_obj.turtle_pose.position.x,turtle_obj.turtle_pose.position.y)
	t1.seth(turtle_obj.turtle_pose.orientation.z)


sub=rospy.Subscriber('turtle',turtle_msg,callback)
image_sub=rospy.Subscriber("image_raw",Image,image_callback, queue_size = 1,buff_size=2**24)


while not rospy.is_shutdown():
	updatepose()							#updatepose based on location of each turtle
	time.sleep(0.01)



