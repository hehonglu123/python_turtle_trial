# RobotRaconteur Single Turtle
## RR Service:
An RR service in this example keeps track of the pose and color of the turtle.
### Service Definition:
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

To start the service, simply run 
```
python turtlebot_service.py
```

## RR Client:
While the service keeps track of the turtle, the client can control the turtle, as well as display the turtle on screen. The **python turtle** module is on client side.
### Keyboard Control:
```
python client_keyboard.py
```
It should bring up a window with a turtle at center, and simply press the arrow key on your keyboard in the terminal to control the turtle.

### Webcam Control:
This example consists of another service of webcam. The client receives the image and process it to detect what color (r/g/b) is at the middle of the image, and then command the turtle according to the color.

To start a webcam service:
```
python webcam_service.py
```
To display the image of the webcam:
```
python streaming_client.py
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
