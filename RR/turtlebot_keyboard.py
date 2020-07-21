import termios, fcntl, sys, os
import turtle
import time
from RobotRaconteur.Client import *     #import RR client library
import traceback
#keyboard reading settings
fd = sys.stdin.fileno()
oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)
oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

#display setup
screen = turtle.Screen()
screen.bgcolor("lightblue")

with RR.ClientNodeSetup(argv=sys.argv):
    url='rr+tcp://localhost:22222/?service=Turtlebot_Service'
    #take url from command line
    if (len(sys.argv)>=2):
        url=sys.argv[1]
    sub=RRN.SubscribeService(url)
    while True:
       try:
           obj = sub.GetDefaultClient()
           turtle_pose_wire=sub.SubscribeWire("turtle_pose_wire")
           break
       except RR.ConnectionException:
           time.sleep(0.1)

    t1=turtle.Turtle()
    t1.shape("turtle")

    def updatepose():                    #set a new pose for turtlebot
        if obj.color=="None":
            t1.penup()
        else:
            t1.pencolor(obj.color)
        if (turtle_pose_wire.TryGetInValue()[0]):
            t1.setpos(turtle_pose_wire.InValue.x,turtle_pose_wire.InValue.y)
            t1.seth(turtle_pose_wire.InValue.angle)
        

    print("Running")
    print("Press Arrow Key to Control Turtle")
    print("Press q to quit")
    try:
        while True:
            try:
                c = sys.stdin.read()
                if "\x1b[A" in c:
                    obj.drive(10,0)            ####Drive forward
                if "\x1b[B" in c:
                    obj.drive(-10,0)           ####Drive backward               
                if "\x1b[C" in c:
                    obj.drive(0,-10)           ####Drive right
                if "\x1b[D" in c:
                    obj.drive(0,10)            ####Drive left
                if "q" in c:
                    break
                updatepose()

            except IOError: pass
    #finish reading keyboard input
    finally:
        termios.tcsetattr(fd, termios.TCSAFLUSH, oldterm)
        fcntl.fcntl(fd, fcntl.F_SETFL, oldflags)