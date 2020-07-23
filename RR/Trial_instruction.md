# Robot Raconteur Survey
Robot Raconteur is an object oriented Service-Client middleware. An RR service generally runs with a hardware (e.g. sensors,actuators) attached to a robot/computer to have direct communication between them. An RR client can receive messages that are sent from services and can call object functions in the service to command the robot. 
In this example, we'll go through how RR webcam streaming works.
# Service Definition
Each RR services have a service definition file, which includes properties, functions and others that are exposed to clients. In other word, the definition inside the `.robdef` file are the ones the client has access to. 
## Webcam Example Service Definition
Let's take a look at the webcam example service definition below:
```
service experimental.createwebcam2

stdver 0.9

struct WebcamImage
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
end

object Webcam
    property WebcamImage image
end
```
First there's a struct for `WebcamImage`, consisting of `width, height, step and data`. The first three elements are the metadata of an image, and the `data` contains the actual pixel information as an 1-D array.
The object that being exposed to the network has the `object` keyword, and in this case it's the `Webcam`. The object in the example only has one member, a property of `WebcamImage` with name `image`. `Property` type is only one value type in RR, in the task below we'll introduce `wire` type, which is a good fit for constantly changing data.

## Create Turtlebot Service Definition
Now let's try create the service definition for a turtlebot:
```
service experimental.turtlebot_create

stdver 0.9

struct pose
    field double x
    field double y
    field double angle
end


object turtlesim
	function void drive(double move_speed, double turn_speed)
	function void setpose(pose turtle_pose)
	wire pose turtle_pose_wire [readonly]
	property string color

end object
```
With provided service definition above, create a file `experimental.turtlebot_create.robdef` under `python_turtle_trial/RR` and copy them into the file as your service definition file. Feel free to modify it. This service definition will give you a sense on what object information to contain in your RR service later.

# RR Service
## Webcam Example Service
Inside `python_turtle_trial/RR`, there's `webcam_service.py` python script for RR webcam service. This example continuously capturing images from a webcam.
At very top, the RobotRaconteur library is imported:
```
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
```
The class `Webcam_impl(object)` is the python object that will be used for RR, but not all of its members are members in service definition. The only member in the service definition, `image`, is defined upon initializaiton:
```
self.image=RRN.NewStructure("experimental.createwebcam2.WebcamImage")
```
Other parts in the class are mostly for continuously streaming of the webcam. Inside `CaptureFrame()` function, the image data is read inside the while loop and formed into the struct defined in service definition:
```
ret, frame=self._capture.read()
if not ret:
    raise Exception("Could not read from webcam")
self.image.width=frame.shape[1]
self.image.height=frame.shape[0]
self.image.step=frame.shape[1]*3
self.image.data=frame.reshape(frame.size, order='C')
```
Now take a look at the main function, the node name and port number is specified in the following line:
```
with RR.ServerNodeSetup("Webcam_Service",2355) as node_setup:
```
In this case, the node name is "Webcam_Service" and the port is 2355.
The service definition file is registered right after that:
```
RRN.RegisterServiceTypeFromFile("experimental.createwebcam2")
```
Then the object is created and the service is registered:
```
c1=Webcam_impl()
RRN.RegisterService("Webcam","experimental.createwebcam2.Webcam",c1)
```
The service is registered with name "Webcam", type of "experimental.createwebcam2.Webcam", actual object of c1

The `input()` function at last holds the script from exiting. To run this script, simply do `$ python webcam_service.py`.


## Create Turtlebot Service
Now let's create an RR service for the turtlebot. Create a new file and name it `turtlebot_service.py`. First import the RR and other essential libraries at top:
```
import numpy as np
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
```
Then create a class `create_turtle`, and fill in according to the service definition:
```
class create_turtle:
	def __init__(self):               			#initialization upon creation
		#RR property
		self.turtle_pose=RRN.NewStructure("experimental.turtlebot_create.pose")	#create RR structure obj
		self.color="None"
	def drive(self,move_speed,turn_speed):            #Drive function, update new position, this is the one referred in definition
		#assign those values
		self.turtle_pose.x=<integrate speed>			
		self.turtle_pose.y=<integrate speed>
		self.turtle_pose.angle=<integrate speed>
		#update wire value
		self.turtle_pose_wire.OutValue=self.turtle_pose
	def setpose(self,turtle_pose):			  #Setpose function, change the pose of turtle directly
		self.turtle_pose=<update pose>
		#update wire value
		self.turtle_pose_wire.OutValue=self.turtle_pose
```
After creating the class object, it's necessary to intialize an RR node as well as registering the service like in *Webcam Example*:
```
with RR.ServerNodeSetup(<service definition name>, <port number>):      #setup RR node with service name and port
	#Register the service type
	RRN.RegisterServiceTypeFromFile(<service definition name>)               #register service type

	create_inst=<create class object>                #create object

	#Register the service with definition and object
	RRN.RegisterService(<service name>,<service definition object>,<object to be passed>)

	#Wait for program exit to quit
	input("Press enter to quit")
```
	
By filling in the sections above, an RR turtlebot service is ready to run!	

# RR Client
## Webcam Example Streaming Client
There's script `streaming_client.py` under `python_trials/RR`. The RR client library is imported at the top:
```
from RobotRaconteur.Client import *
```
The function `WebcamImageToMat(image)` converts RR image property to openCV object. Inside main function, the `url` is the string containing the IP address of the service, the port the service is on and the service name. The default url in script is
```
url='rr+tcp://localhost:2355/?service=Webcam'
```
But it's possible to use command line options to provide the url:
```
$ python streaming_client.py rr+tcp://localhost:2355/?service=Webcam
```

,where `localhost` means the service is running on this local machine.
```
cam=RRN.ConnectService(url)
```
`cam` is the object returned by `ConnectService`, and it should contain the member defined in the service definition, which is the `image` property. And the image is accessed and converted to openCV object by
```
WebcamImageToMat(cam.image)
```
To run this script, simply do `$ python streaming_client.py`.

## Create Simple Turtlebot Client
Now let's create an RR keyboard control client for the turtlebot. Create a new file and name it `turtlebot_client.py`. First import the RR and other essential libraries at top:
```
from RobotRaconteur.Client import *     #import RR client library
import turtle
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
After that, it's necessary to have an `update` function to update the display:
```
def update(obj):                    	#Update the display
        if obj.color=="None":		#update pen color on display
            t1.penup()
        else:
            t1.pencolor(obj.color)

	t1.setpos(obj.turtle_pose_wire.PeekInValue()[0].x,obj.turtle_pose_wire.PeekInValue()[0].y)
	t1.seth(obj.turtle_pose_wire.PeekInValue()[0].angle)
```
Then comes the RR part, we intialize the RR client node and connect to the service:
```
with RR.ClientNodeSetup(argv=sys.argv):
	url='rr+tcp://localhost:<port number>/?service=<service name>'
	#take url from command line
	if (len(sys.argv)>=2):
		url=sys.argv[1]
	turtle_obj=RRN.ConnectService(url)
```
A good example would be having the turtle driving a circle:
```
	while True:
		turtle_obj.drive(10,10)
		update()		#update in display
```


# Task
From the tutorial above, you should have a complete turtlebot service and a simple turtlebot client. Given the example of reading keyboard inputs under `python_turtle_trial/Examples`, try adapt it into an RR client that can control the turtle through keyboard.

Given the camera service `RR/webcam_service.py` and detection example `Examples/detection.py`, try create a client reading in images from the webcam service, process the image and drive the turtle based on the color detected in your webcam.
