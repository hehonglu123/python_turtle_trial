# ROS Python Turtle Example
## Build ROS package:
The package is already generated. To build it along with messages and services, 
```
cd ~/python_turtle_trial/ROS/
catkin_make
echo 'source ~/python_turtle_trial/ROS/devel/setup.bash' >> ~/.bashrc 
```
It only needs to be done once, except there's modification on the service/message file.

## ROS server:
The script `turtlebot_service` keeps track of the turtle pose and color.
### Message definition:
Under `msg/` directory, the `turtle.msg` defines the message type `turtle`:
```
string name
geometry_msgs/Pose turtle_pose
string color
```
### Service definition:
Under `srv/` directory, the `setcolor.srv` defines the service function of `setcolor`, and the `setpose.srv` defines the service function of `setpose`.

### Usage:
To start the server, it's necessary to start the `roscore` first:
```
roscore
```
Then just open a new terminal and run 
```
python turtlebot_service.py
```

## ROS client:
### Keyboard Control:
```
python turtlebot_keyboard.py
```
It should bring up a window with a turtle at center, and simply press the arrow key on your keyboard in the terminal to control the turtle.

### Webcam Control:
This example consists of another server of webcam. The client receives the image and process it to detect what color (r/g/b) is at the middle of the image, and then publish the command according to the color.

To start a webcam publisher:
```
python cam_pub.py
```
To display the image of the webcam:
```
python cam_sub.py
```
To start the turtle service:
```
python turtlebot_service.py
```
To start the final client:
```
python client_color.py
```
Point the camera toward red/green/blue objects to drive the turtle.
