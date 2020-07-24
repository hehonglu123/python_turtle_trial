import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from python_turtle.msg import turtle_msg
from python_turtle.srv import setcolor, setpose


#Actual class object
class create_turtle:
	def __init__(self):               			#initialization upon creation
		#message type initialziation
		self.turtle_pose=Pose()
		self.turtle=turtle_msg()
		#subscriber initialization
		self.vel_sub=rospy.Subscriber('drive',Twist,self.drive_callback)
		#publisher initialization
		self.turtle_pub=rospy.Publisher('turtle',turtle_msg,queue_size=1)
		#service initialization
		self.pose_server=rospy.Service('setpose', setpose, self.set_pose)
		self.color_server=rospy.Service('setcolor', setcolor, self.set_color)
	def drive_callback(self,data):            #Drive function, update new position, this is the one referred in definition
		self.turtle.turtle_pose.position.x+=data.linear.x*np.cos(np.radians(self.turtle.turtle_pose.orientation.z))
		self.turtle.turtle_pose.position.y+=data.linear.x*np.sin(np.radians(self.turtle.turtle_pose.orientation.z))
		self.turtle.turtle_pose.orientation.z+=data.angular.z

	def set_pose(self,req):		#update pose based on given pose
		self.turtle.turtle_pose=req.turtle_pose.pose
		return 1
	def set_color(self,req):	#set color based on given color
		self.turtle.color=req.color
		return 1
	


rospy.init_node('turtle_server_node', anonymous=True)
turtle_obj=create_turtle()
print("running")
while not rospy.is_shutdown():
	turtle_obj.turtle_pub.publish(turtle_obj.turtle)
	time.sleep(0.01)