# RobotRaconteur turtle module example
![](images/turtle.gif)

## Prerequisite:
### python3
### RobotRaconteur

## Usages:
### Start RR Service:
`python3 turtlebot_service.py`
### Start RR Client:
#### Start displaying client: 
`python3 client_display.py`
#### Draw an 'R': 
`python3 client_R.py`
#### Keyboard Control: 
`python3 turtlebot_keyboard.py`

## Service Definition:
```
struct pose
    field double x
    field double y
    field double angle
end
struct turtle
	field string name
	field pose turtle_pose
	field string color
end

object turtlesim
	function void drive(string turtle_name, double move_speed, double turn_speed)
	property turtle{list} turtles
	property bool turtle_change
	function void add_turtle(string turtle_name)
	function void remove_turtle(string turtle_name)
	function void setpose(string turtle_name,pose desire_pose)
	function void setcolor(string turtle_name, string color)
end object
```
# ROS turtle module example

## Prerequisite:
### python or python3
### ROS

## Setup:
```
cd /RR_turtle_RR_ROS/ROS
catkin_make
source /RR_turtle_RR_ROS/ROS/devel/setup.bash		#needed for every new terminal
```

## Run:
`cd /RR_turtle_RR_ROS/ROS/src/python_turtle/src`
### roscore
Open a terminal with `roscore`
### Start ROS Server:
`python turtlebot_service.py`
### Start ROS Client:
#### Start displaying client: 
`python client_display.py`
#### Keyboard Control: 
`python turtlebot_keyboard.py`

## Message Types:
### turtle:
```
string name
geometry_msgs/Pose turtle_pose
string color
```
### turtle_array:
```
turtle[] turtles
```

## Topics:
`/drive`: `TwistStamped`, turtle name at header.frame_id

`/change`: `Int8`, published only when a turtle is added or deleted

`/turtles`: `turtle_array`, a list of current turtles in the scene
## Services:
`/addturtle`: add turtle in the scene given `String` turtle_name

`/removeturtle`: remove turtle in the scene given `String` turtle_name

`/setcolor`: set the pen color of the turtle given `String` turtle_name and color

`/setpose`: set the pose of the turtle give `PoseStamped` with turtle_name in header and pose 
