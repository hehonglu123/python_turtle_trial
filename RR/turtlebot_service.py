import numpy as np
#import RR library
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s

#Actual class object
class create_impl:
	def __init__(self):               			#initialization upon creation

		#RR property
		self.turtle_pose=RRN.NewStructure("experimental.turtlebot_create.pose")
		self.color="None"

	def drive(self,move_speed,turn_speed):            #Drive function, update new position, this is the one referred in definition
		self.turtle_pose.x+=move_speed*np.cos(np.radians(self.turtle_pose.angle))
		self.turtle_pose.y+=move_speed*np.sin(np.radians(self.turtle_pose.angle))
		self.turtle_pose.angle+=turn_speed
		self.turtle_pose_wire.OutValue=self.turtle_pose
	def setpose(self,turtle_pose):
		self.turtle_pose=turtle_pose
		self.turtle_pose_wire.OutValue=self.turtle_pose

	
if __name__ == '__main__':
		
	with RR.ServerNodeSetup("experimental.turtlebot_create", 22222):      #setup RR node with service name and port
		#Register the service type

		RRN.RegisterServiceTypeFromFile("experimental.turtlebot_create")               #register service type

		create_inst=create_impl()                #create object

		#Register the service with definition and object
		RRN.RegisterService("Turtlebot_Service","experimental.turtlebot_create.turtlesim",create_inst)

		#Wait for program exit to quit
		input("Press enter to quit")