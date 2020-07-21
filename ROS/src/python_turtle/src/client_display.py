import turtle as tt
import traceback
import time
import rospy		#import ROS client library
from python_turtle.msg import turtle,turtle_array
from std_msgs.msg import Int8

turtles=turtle_array()
change=1

def callback(data):
	global turtles
	turtles=data
def change_callback(data):
	global change
	change=1


screen = tt.Screen()
screen.bgcolor("lightblue")
#ROS initialization
rospy.init_node('display', anonymous=False)
sub=rospy.Subscriber('turtles',turtle_array,callback)
change_sub=rospy.Subscriber('change',Int8,change_callback)



def update_dict():
	global turtles
	turtle_dict={}						#turtle_dict: names<->turtle object
	for i in range(len(turtles.turtles)):
		turtle_dict[turtles.turtles[i].name] = tt.Turtle()
		turtle_dict[turtles.turtles[i].name].shape("turtle")
	return turtle_dict

turtle_dict=update_dict()
def updatepose():           		 #set a new pose for turtlebot
	global turtle_dict, turtles
	for i in range(len(turtles.turtles)):
		if turtles.turtles[i].color=="None":
			turtle_dict[turtles.turtles[i].name].penup()
		else:
			turtle_dict[turtles.turtles[i].name].pendown()
			turtle_dict[turtles.turtles[i].name].pencolor(turtles.turtles[i].color)
		turtle_dict[turtles.turtles[i].name].setpos(turtles.turtles[i].turtle_pose.position.x,turtles.turtles[i].turtle_pose.position.y)
		turtle_dict[turtles.turtles[i].name].seth(turtles.turtles[i].turtle_pose.orientation.z)

while not rospy.is_shutdown():
	global change
	try:
		if change==1:
			screen.clearscreen()
			turtle_dict=update_dict()
			screen.bgcolor("lightblue")
			change=0
		updatepose()							#updatepose based on location of each turtle
		time.sleep(0.01)
	except KeyError:
		continue
	except IndexError:
		continue
	except:
		traceback.print_exc()
		break


