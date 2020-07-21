import termios, fcntl, sys, os
import rospy     #import ROS library
import turtle as tt
from geometry_msgs.msg import TwistStamped
from python_turtle.msg import turtle


turtle_obj=turtle()
def callback(data):
    global turtle_obj
    turtle_obj=data

    

#keyboard reading settings
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

#display setup
screen = tt.Screen()
screen.bgcolor("lightblue")
t1=tt.Turtle()
t1.shape("turtle")

def updatepose():                    #set a new pose for turtlebot
    global turtle_obj
    if turtle_obj.color=="None":
        t1.penup()
    else:
        t1.pencolor(turtle_obj.color)

    t1.setpos(turtle_obj.turtle_pose.position.x,turtle_obj.turtle_pose.position.y)
    t1.seth(turtle_obj.turtle_pose.orientation.z)

#ROS initialization
rospy.init_node('keyboard_control', anonymous=False)
pub=rospy.Publisher('drive',TwistStamped,queue_size=1)
sub=rospy.Subscriber('turtle',turtle,callback)



print("Running")
print("Press Arrow Key to Control Turtle")
print("Press q to quit")
try:
    while not rospy.is_shutdown():
        try:
            #update turtle pose on screen
            updatepose() 
            #form ros msg
            msg=TwistStamped()
            c = sys.stdin.read()
            if "\x1b[A" in c:
                msg.twist.linear.x=10                   ####Drive forward
            if "\x1b[B" in c:
                msg.twist.linear.x=-10                  ####Drive backward               
            if "\x1b[C" in c:
                msg.twist.angular.z=-10                 ####Drive right
            if "\x1b[D" in c:
                msg.twist.angular.z=10                  ####Drive left
            if "q" in c:
                break
            pub.publish(msg)     
            
        except IOError: pass
#finish reading keyboard input
finally:
    termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)

