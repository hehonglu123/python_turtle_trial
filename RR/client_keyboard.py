import termios, fcntl, sys, os
import turtle
import time
from keyboard import _Getch
from RobotRaconteur.Client import *     #import RR client library
import traceback

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
	
	getch = _Getch()
	

	while True:
		c=getch()
		if "A" in c:
			obj.drive(10,0)            ####Drive forward
		if "B" in c:
			obj.drive(-10,0)           ####Drive backward               
		if "C" in c:
			obj.drive(0,-10)           ####Drive right
		if "D" in c:
			obj.drive(0,10)            ####Drive left
		if "q" in c:
			break
		updatepose()
