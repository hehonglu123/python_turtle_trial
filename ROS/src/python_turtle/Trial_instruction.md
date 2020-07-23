# ROS Survey
The structure of ROS is a little different from Robot Raconteur. It has the Publisher-Subscriber relationship between different nodes.

# Setup
## Catkin Workspace
For each ROS project there's a catkin workspace dedicated, and in this trial the workspace is `python_turtle_trial/ROS`.
## Package
Unlike RobotRaconteur, ROS requires the workspace to build the content. All packages should be in `workspace/src/` folder. In this repository there's already a webcam package, so you'll need to [create another package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) for python turtle:
```
cd ~/python_turtle_trial/ROS
catkin_create_pkg python_turtle std_msgs geometry_msgs rospy
```
This creates a new package under `cd ~/python_turtle_trial/ROS/src`, with dependency of `std_msgs`, `geometry_msgs` and `rospy`.

# Message Types
Similar to RobotRaconteur service definition, for ROS there're [message types](http://wiki.ros.org/Messages) and [service types](http://wiki.ros.org/Services). In the task we'll need to create our own message and service types.
When building ROS, many message and service types are built together, which you can look up online: http://wiki.ros.org/common_msgs. 
In the task we'll ask to create your own message type `turtle_ros`:
```
string name
geometry_msgs/Pose turtle_pose
string color
```
This bascially shows the message contains the name of the turtle, its pose and color. Since this message is part of the `python_turtle` package, create a folder under `python_turtle_trial/ROS/src/python_turtle/` called `msg`. Then create a file named `turtle_msg`, and copy above message definition into this file as your own message type.

In order to let the compiler know and built the message for you, it's necessary to modify the `package.xml` as well as `CMakeLists.txt` under package `python_turtle`. So first open up `package.xml` and uncomment below two lines:
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
Then open up `CMakeLists.txt`, search for lines below:
```
find_package(catkin REQUIRED COMPONENTS
   rospy
   std_msgs
   geometry_msgs
)
```
Add `message_generation` into `find_package`.

To export the message runtime dependency, inside `CMakeLists.txt` look for 
```
catkin_package(
...
)
```
Add `CATKIN_DEPENDS message_runtime` within `catkin_package`
Then find the following block of code:
```
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
```
Change it to 
```
add_message_files(
  FILES
  turtle_msg.msg
)
```
Finally, look for lines below:
```
## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
...
# )
```
And change them to
```
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)
```
By this point, the message type should be built together when building the package.

You can include the message type in the similar way of ROS Image type:
```
from python_turtle import turtle_msg
from geometry_msgs import Pose
```
And to create an object of that message type:
```
turtle_obj=turtle_msg()
turtle_obj.name="myturtle"
turtle_obj.turtle_pose=Pose()
turtle_obj.color="red"
```

# Service Types
A ROS service is similar to a function call, and in the task we'll ask you to create a `setpose` and `setcolor`:
`setpose`:
```
geometry_msgs/PoseStamped turtle_pose
---
int8 ret
```
`setcolor`:
```
string color
---
int8 ret
```
ROS service has to have a return type, so we can simply return an `int` instead of `void`. 

Generating a ROS service type is similar to generating a message type. First create a folder under `python_turtle_trial/ROS/src/python_turtle/` called `srv`. Then create two files named `setpose` and `setcolor`, and copy above service definitions into these files as your own service types.
Some steps are overlapped when creating services and messages, so only a bit differences. Navigate to `CMakeLists.txt` under package `python_turtle`, open it up and look for code below:
```
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
``` 
And modify it to
```
add_service_files(
  FILES
  setcolor.srv
  setpose.srv
)
```

To set up a service, it's necessary to initialize it first:
```
self.pose_server=rospy.Service('setpose', setpose, self.set_pose)
```
and the function is provided later in the same class:
```
def set_pose(self,req):
	self.turtle.turtle_pose=req.turtle_pose.pose
	return 1
```
Remember to build your workspace and source it to get your service types exposed.

## build workspace
Type in following commands to build your workspace:
```
cd ~/python_turtle_trial/ROS
catkin_make
```
It should finish without errors, and generating `/build` and `/devel` folders under the same directory. However, to make sure your code knows what you've built, we need to source it:
```
$ echo 'source ~/python_turtle_trial/ROS/devel/setup.bash' >> ~/.bashrc 
```
This step adds the command everytime you open up a new terminal. If errors like something not found or not built, try `source ~/python_turtle_trial/ROS/devel/setup.bash` to source the workspace directly.

# ROS Publisher
Under `~/python_turtle_trial/ROS/src/webcam/src/` there is a python script called `cam_pub.py`. At the very top, we include ROS library and message types:
``` 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
```
The `Webcam_impl()` class is a webcam class, which contains camera metadata and a `CaptureFrame()` function. Then take a look at `main`:
```
pub = rospy.Publisher('image_raw', Image, queue_size=0) 
rospy.init_node('picam', anonymous=True) 
```
Here ROS node is initialized with a publisher, published to topic `image_raw` of type `Image` ([sensor_msgs/Image.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
```
while not rospy.is_shutdown(): 
```
This while loop holds the sciprt from exiting until ROS is shutdown, and inside the loop:
```
frame=picam.CaptureFrame() 
pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8")) 
```
The image is captured and convert to ROS Image type, and finally published to the topic by the publisher.
To run this script, open a new terminal and run `roscore`. After that, you can run this script by `python cam_pub.py`.
For every ROS communication, there needs to be one and only one roscore running. To check if the images are successfully published or not, open up a new terminal and type in `rostopic echo image_raw`.
This way the terminal shall display the image data.

# ROS Subscriber

On your computer side, under `~/python_turtle_trial/ROS/src/webcam/src/` there is a python script called `cam_sub.py`. We include ROS library and message types at the top.
Unlike a publisher, a subscriber subscribe to the topic, and trigger the `callback()` function. Inside main, 
```
rospy.init_node('stream_node', anonymous=True)
sub = rospy.Subscriber("image_raw",Image,callback)
rospy.spin()
```
ROS node is intialized, and a subscriber `sub` is set up to subscribe to ROS topic `image_raw`, with `Image` type, triggering `callback()` function. `rospy.spin()` keeps this script running until user shutdown. Now let's take a look at the `callback()` function.
`def callback(data)` means this function takes in an argument of `data`, which should be the message type specified in the subscriber setup (`Image`). 
```
try:
	cv_image = bridge.imgmsg_to_cv2(data, "bgr8")	#convert ros image message to opencv object
except CvBridgeError as e:
	print(e)
```
Above lines basically convert the `Image` type data into an openCV image object, so that it could be displayed out on screen.




## Create Turtlebot Server
All script about turtles should be in `~/python_turtle_trial/ROS/src/python_turtle/src/` directory, so let's create a file named `turtlebot_service.py`. The function of this script is to keep track of the turtle (pose and color).
First, it's necessary to include ROS and relative libraries and messages:
```
import time
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose
from python_turtle.msg import turtle_msg
from python_turtle.srv import setcolor, setpose
```
Here `Twist` and `Pose` messages are two standard ones in [geometry messages](http://wiki.ros.org/geometry_msgs), and `turtle_msg` is the one we created earlier.
Then let's also create a python class object called `create_turtle`, and initialize ROS publisher, subscriber and services:
```
class create_turtle:
	def __init__(self):               			#initialization upon creation
		#message type initialziation
		self.turtle_pose=Pose()
		self.turtle=turtle_msg()
		#subscriber initialization
		self.vel_sub=rospy.Subscriber(<drive topic name>,<message type>,self.drive_callback)
		#publisher initialization
		self.pose_pub=rospy.Publisher(<turtle topic name>,<message type>,queue_size=1)
		#service initialization
		self.pose_server=rospy.Service(<service name>, <service type>, self.set_pose)
		self.color_server=rospy.Service(<service name>, <service type>, self.set_color)
```
Note that in the service there're function `drive_callback`, `set_pose` and `set_color`, therefore we'll also need to implement those so when a service is called or a subscriber receives message, the function will be triggered. Inside the same class,
```
	def drive_callback(self,data):            #Drive function, update new position, this is the one referred in definition
		self.turtle.turtle_pose.position.x=<decode message type and update pose>
		self.turtle.turtle_pose.position.y=<decode message type and update pose>
		self.turtle.turtle_pose.orientation.z=<decode message type and update pose>

	def set_pose(self,req):		#update pose based on given pose
		self.turtle.turtle_pose=req.turtle_pose.pose
		return 1
	def set_color(self,req):	#set color based on given color
		self.turtle.color=req.color
		return 1
```
After having a complete class, it's necessary to initialize a ROS node as well as create the class object:
```
rospy.init_node('turtlebot', anonymous=True)
turtle_obj=create_turtle()
```
Note that the goal for this script is to keep track of the turtle, so we also need to publish the turtle pose continuously.
```
while not rospy.is_shutdown():
	turtle_obj.pose_pub.publish(turtle_obj.turtle)
	time.sleep(0.01)
```
By filling up the `<>` sections above, you should have a complete turtlebot server.

## Create Turtlebot Client
Now let's create a simple ROS client for the turtlebot. Create a new file and name it `turtlebot_client.py`. First import the ROS and other essential libraries at top:
```
import rospy     #import ROS library
import turtle as tt
from geometry_msgs.msg import TwistStamped
from python_turtle.msg import turtle_msg
```
Similar to the `turtlebot.py`, we need to initialize the screen and a turtle first:
```
#display setup
screen = turtle.Screen()
screen.bgcolor("lightblue")
#turtle setup
t1=turtle.Turtle()
t1.shape("turtle")
```
Then it's necessary to create a variable keeping the turtle pose information from ROS subscriber callback:
```
turtle_obj=turtle_msg()
def callback(data):
	global turtle_obj
	turtle_pose=data
```
And it's also necessary to have an `update()` function to update display:
```
def update()
	global turtle_obj
	if turtle_obj.color=="None":
		t1.penup()
	else:
		t1.pencolor(<color>)

	t1.setpos(<x coordinate>,<y coordinate>)
	t1.seth(<angle>)
```
Finally, we initialize ROS node as well as the publisher and subscriber:
```
rospy.init_node(<random node name>, anonymous=False)
pub=rospy.Publisher(<drive topic name>,<message type>,queue_size=1)
sub=rospy.Subscriber(<turtle topic name>,<message type>,callback)
```
After ROS is initialized, let's also have the turtle running a circle:
```
while not rospy.is_shutdown():
        update()
	msg=Twist()	#create message object
	msg.linear.x=10
	msg.angular.z=10
	pub.publish(msg)  
```
All rospy scripts can run with `python` command, but make sure have one and only one `roscore` running.

# Task
From tutorial above, you should have a complete turtlebot subscriber and a simple turtlebot publisher. Given `keyboard.py` under `python_turtle_trial/Examples`, try creating a client that display the turtle as well as reading in inputs from the keyboard to drive the turtle accordingly.


Given the camera publisher `ROS/cam_pub.py` and detection example `Examples/detection.py`, try create a client subscribing images from the webcam, process the image and publishing command to drive the turtle based on the color detected in your webcam.
