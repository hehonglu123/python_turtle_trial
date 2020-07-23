import time
import numpy as np
#import ROS library
import rospy
from std_msgs.msg import Int8
from geometry_msgs.msg import TwistStamped, Pose
from python_turtle.msg import turtle
from python_turtle.srv import setcolor, setpose


#Actual class object
class create_impl:
	def __init__(self):               			#initialization upon creation
		#message type initialziation
		self.turtle_pose=Pose()
		self.turtle=turtle()

		self.vel_sub=rospy.Subscriber('drive',TwistStamped,self.drive_callback)

		self.pose_server=rospy.Service('setpose', setpose, self.set_pose)
		self.color_server=rospy.Service('setcolor', setcolor, self.set_color)

	def drive_callback(self,data):            #Drive function, update new position, this is the one referred in definition
		self.turtle.turtle_pose.position.x+=data.twist.linear.x*np.cos(np.radians(self.turtle.turtle_pose.orientation.z))
		self.turtle.turtle_pose.position.y+=data.twist.linear.x*np.sin(np.radians(self.turtle.turtle_pose.orientation.z))
		self.turtle.turtle_pose.orientation.z+=data.twist.angular.z

	def set_pose(self,req):
		self.turtle.turtle_pose=req.turtle_pose.pose
		return 1
	def set_color(self,req):
		self.turtle.color=req.color
		return 1
	
		

if __name__ == '__main__':
		
	rospy.init_node('turtlebot', anonymous=True)
	turtle_obj=create_impl()
	pose_pub=rospy.Publisher('turtle',turtle,queue_size=1)
	print("running")
	while not rospy.is_shutdown():
		pose_pub.publish(turtle_obj.turtle)
		time.sleep(0.01)
