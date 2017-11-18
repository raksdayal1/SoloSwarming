#!/usr/bin/env python
import rospy
import math
import serial
import time
import sys

import threading
import NodeDeps

from numpy import matrix, linalg, sort, nonzero
from scipy.sparse.csgraph import csgraph_from_dense, laplacian, csgraph_to_dense

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from swarm_control.msg import State, actions

from swarm_control.srv import *
from dronekit import connect, LocationLocal

class ROSGCS:
	def __init__(self, ComPort='/dev/ttyACM0'):
		rospy.init_node('GCS', anonymous=True)
		
		self.speed_rate = 10.0
		self.rate = rospy.Rate(self.speed_rate)
		self.dt = 1.0/self.speed_rate

		self.no_nodes = NodeDeps.Init_Graph()[2]
		
		# connect to the apm attached and get GPS lock
		self.GCS=connect(ComPort, wait_ready=False)
		while not self.GCS.gps_0.fix_type == 3:
			print('GCS: Waiting to get GPS lock ')
			time.sleep(3)
		
		self.CommonHome = self.GCS.location.global_frame
		self.nav = NodeDeps.NavFrame(self.CommonHome.lat, self.CommonHome.lon, self.CommonHome.alt)

		print('GCS: Got GPS lock on Ground Station lat=%f, lon=%f, alt=%f' % (self.GCS.location.global_frame.lat,self.GCS.location.global_frame.lon,self.GCS.location.global_frame.alt))
		
		# Create setHome service
		Homeservice = rospy.Service('/GCS/Home', HomeMsg, self.SetHomeService)

		# Create the comm paths
		self.CreateComms()
		time.sleep(1)

		self.MakeFormation()
		print('GCS: Start all flight nodes. GCS sleeping for 2 minutes')
		time.sleep(120)
		print('GCS: Waking up!!!')		

                # FUTURE:raw_input isnt working. Check out why
		StartFlag = 'yes'#raw_input('GCS: Type yes if flight path is uploaded:= ')
		if StartFlag == 'yes':
			print('GCS: Ok! Sleeping for 5 seconds before arming and takeoff')
			time.sleep(5)
			
			# Publish arm and take off commands
			msg = actions()

			msg.arm.data = True
			msg.takeoff.data = True
			msg.land.data = False

			print('GCS: Publishing arming and takeoff commands')
			for n in xrange(0,self.no_nodes):
				self.pub_actions[n].publish(msg)
				self.rate.sleep()

			time.sleep(10)
			print('GCS: Starting formation. Launch the swarm controller')
			self.RunFormation()

		else:
			sys.exit(1)
			

		
	def CreateComms(self):
		self.pub_form = [0]*self.no_nodes
		self.pub_actions = [0]*self.no_nodes

		#self.homeServices = [0]*self.no_nodes
	
		for n in xrange(0,self.no_nodes):
			self.configtopic_name = 'solo' + str(n+1) + '/formconfig'
			self.pub_form[n] = rospy.Publisher(self.configtopic_name, Twist, queue_size=10, latch=True) # change from Twist to Pose

			self.actions_name = 'solo' + str(n+1) + '/Commands/Actions'
			self.pub_actions[n] = rospy.Publisher(self.actions_name, actions, queue_size=10, latch=True) 

			#self.homeServices[n] ='solo' + str(n+1) + '/setHome'
		
		print('GCS: Communication paths ready')


	def SetHomeService(self, req):
		return HomeMsgResponse(self.CommonHome.lat, self.CommonHome.lon, self.CommonHome.alt)


	def MakeFormation(self):
		# This formation geometry should come from an external file
		self.Formation_geo = matrix([[2, -2],[0, 0],[-6, -3]]) # always set formation in NED always d is -ve 
		if not self.Formation_geo.shape[1] == self.no_nodes:
			sys.exit(-1)
		

	def RunFormation(self):	
		# Get all waypoints
		cmds = self.GCS.commands
		cmds.download()
		cmds.wait_ready()

		NEDWp = self.Convert2NEDMission(cmds)
		x_0 = NEDWp[1].north
		y_0 = NEDWp[1].east
		z_0 = NEDWp[1].down
		Wp_no = 1

		msg = Twist()
		while not rospy.is_shutdown():
			#print "Waypoint no ", Wp_no
			while not distance2point(x_0,y_0,z_0,NEDWp[Wp_no+1]) < 2:
				v_x = saturate(NEDWp[Wp_no+1].north - x_0, 1)
				v_y = saturate(NEDWp[Wp_no+1].east - y_0, 1)
				v_z = saturate(NEDWp[Wp_no+1].down - z_0, 1)
				

				x_0 = euler(v_x, x_0, self.dt)
				y_0 = euler(v_y, y_0, self.dt)
				z_0 = euler(v_z, z_0, self.dt)
				#print x_0, y_0, z_0
				#print distance2point(x_0,y_0,z_0,NEDWp[Wp_no+1])

				for n in xrange(0,self.no_nodes):
					msg.linear.x = self.Formation_geo[0,n] + x_0
					msg.linear.y = self.Formation_geo[1,n] + y_0
					msg.linear.z = self.Formation_geo[2,n] + z_0
						
					self.pub_form[n].publish(msg)
				#time.sleep(2)
				self.rate.sleep()

			time.sleep(10)
			Wp_no +=1
			if Wp_no == cmds.count-2:
				print "Ending"
				break
			
			print("GCS: Landing initiated ")
			self.Landing()


	def Landing(self):
		# Publish arm and take off commands
		msg = actions()

		msg.arm.data = True
		msg.takeoff.data = False
		msg.land.data = True

		print('GCS: Publishing arming and takeoff commands')
		for n in xrange(0,self.no_nodes):
			self.pub_actions[n].publish(msg)
			self.rate.sleep()
		time.sleep(10)

			


	def Convert2NEDMission(self, Commands):
		NED_Wp =[]
		
		for seq_no in xrange(1,Commands.count):
			
			# read only waypoints information
			if Commands[seq_no].command == 16 and Commands[seq_no].frame == 3:
				# Get the LLA with relative alt. Add to home alt and convert to NED
				ned = self.nav.LLA2NED(Commands[seq_no].x, Commands[seq_no].y, self.nav.Home.alt + Commands[seq_no].z)
			elif Commands[seq_no].command == 16 and Commands[seq_no].frame == 0:
				# Get the LLA with relative alt. Add to home alt and convert to NED
				ned = self.nav.LLA2NED(Commands[seq_no].x, Commands[seq_no].y, Commands[seq_no].z)

			if Commands[seq_no].command == 22 and Commands[seq_no].frame == 3:
				# Get the LLA with relative alt. Add to home alt and convert to NED
				ned = self.nav.LLA2NED(Commands[seq_no].x, Commands[seq_no].y, self.nav.Home.alt + Commands[seq_no].z)
			elif Commands[seq_no].command == 22 and Commands[seq_no].frame == 0:
				# Get the LLA with relative alt. Add to home alt and convert to NED
				ned = self.nav.LLA2NED(Commands[seq_no].x, Commands[seq_no].y, Commands[seq_no].z)

			NED_Wp.append(ned)
		return NED_Wp

def saturate(Val, lim):
	if Val > lim:
		Val = lim
	elif Val < -lim:
		Val = -lim
	return Val


def distance2point(x, y, z ,disNEDWp):
	dis_x = disNEDWp.north - x
	dis_y = disNEDWp.east - y
	dis_z = disNEDWp.down - z
	return math.sqrt(dis_x**2 + dis_y**2 +dis_z**2)

def euler(f_x,x_prev,dt):
	x_cur = x_prev + f_x*dt
	return x_cur




#if __name__ == "__main__":
time.sleep(1)
print('Starting Ground Control Station. Connecting to attached device to run HOME sequence')

MasterPort='/dev/ttyACM0, 115200' # serialport to which the GPS device is connected
RosInterfacePort='127.0.0.1:'+str(17255) 
QGCInterfacePort='192.168.1.105:'+str(8550)#'192.168.0.155:'+str(8550) #IP of system running QGC

t=threading.Thread(target=NodeDeps.launchMAVProxy, args=(MasterPort, RosInterfacePort, QGCInterfacePort,))
t.setDaemon(False)
t.start()

time.sleep(5)
ROSGCS(ComPort='udpin:'+RosInterfacePort)
