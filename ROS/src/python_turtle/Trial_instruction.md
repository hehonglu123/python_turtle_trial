# ROS Survey
The structure of ROS is a little different from Robot Raconteur. It has the Publisher-Subscriber relationship between different nodes.

# Setup
## package
Unlike RobotRaconteur, ROS requires a special workspace to build the content. In this repository there's already a webcam package, so you'll need to [create another package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) for python turtle:
```
cd ~/python_turtle_trial/ROS
catkin_create_pkg python_turtle std_msgs geometry_msgs rospy
```
This creates a new package under `cd ~/python_turtle_trial/ROS/src`, with dependency of `std_msgs`, `geometry_msgs` and `rospy`.

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
Under `~/python_turtle_trial/ROS/src/python_turtle/src/` there is a python script called `cam_pub.py`. At the very top, we include ROS library and message types:
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
On your computer side, under `~/python_turtle_trial/ROS/src/python_turtle/src/` there is a python script called `cam_sub.py`. Similarly, we include ROS library and message types at the top.
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
	print(e)```
Above lines basically convert the `Image` type data into an openCV image object, so that it could be displayed out on screen.


# Message Types
Similar to RobotRaconteur service definition, for ROS there're [message types](http://wiki.ros.org/Messages) and [service types](http://wiki.ros.org/Services). In the task we'll need to create our own message and service types.
When building ROS, many message and service types are built together, which you can look up online: http://wiki.ros.org/common_msgs. 
In the task we'll ask to create your own message type `turtle_ros`:
```
string name
geometry_msgs/Pose turtle_pose
string color
```
This bascially shows the message contains the name of the turtle, its pose and color.
Follow the instructions on http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv to create this message.

You can include the message type in the similar way of ROS Image type:
```
from python_turtle import turtle_ros
from geometry_msgs import Pose
```
And to create an object of that message type:
```
turtle_obj=turtle_ros()
turtle_obj.name="myturtle"
turtle_obj.turtle_pose=Pose()
turtle_obj.color="red"
```

Remember to build your workspace and source it to get your message type exposed.

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
ROS service has to have a return type, so we can simply return an `int` instead of `void`. Follow the instructions on http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv to create the services.

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

# Task
## 1
Given the example of `turtlebot.py` and `keyboard.py`, create a turtlebot server keeping track of the pose of the turtlebot. The message and service types are already provided, and make sure to build your workspace and source it everytime there's a change in service or message. 

If the server runs without any error, try creating a client that display the turtle as well as reading in inputs from the keyboard to drive the turtle accordingly.

## 2
Given the camera publisher `ROS/cam_pub.py` and detection example `Examples/detection.py`, try create a client subscribing images from the webcam, process the image and publishing command to drive the turtle based on the color detected in your webcam.
