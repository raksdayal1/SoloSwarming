#!/usr/bin/env python
import rospy
import threading
import time
import logging

from swarm_control.msg import gps_ins, command, actions
from geometry_msgs.msg import Pose, Twist


class SwarmController:
	def __init__(self):
		rospy.init_node('Controller', anonymous=True)
		# Publishers '
		ns = rospy.get_namespace()
		self.pub = rospy.Publisher('Commands/ControlPose', command, queue_size=10)

		# Subscribers
		rospy.Subscriber('command/cmd_vel', Twist, self.Cmd_callback)
		rospy.Subscriber('Sensors/GPS_INS', gps_ins, self.Odem_callback)
	
		# Rate
		self.speed_rate = 50.0
		self.rate = rospy.Rate(self.speed_rate)
		time.sleep(10)
		self.RunControl()
	

	def RunControl(self):
		self.rate.sleep()	
		
		# Set up initial conditions
		x_0 = 0#self.pos_msg.north
		y_0 = 0#self.pos_msg.east
		z_0 = 0#self.pos_msg.down
		vx_0 = 0#self.twist_msg.x
		vy_0 = 0#self.twist_msg.y
		vz_0 = 0#self.twist_msg.z
		
		dt = 1.0/self.speed_rate
		msg = command()
		
		msg.ControlType = 0

		while not rospy.is_shutdown():
			try:
				v_x = self.saturate(self.cmd.linear.x, 5)
				v_y = self.saturate(self.cmd.linear.y, 5)
				v_z = self.saturate(self.cmd.linear.z, 5)
			except:
				v_x = 0
				v_y = 0
				v_z = 0
			
				
			if msg.ControlType == 1: # publish position commands
				x_0 = self.euler(v_x, x_0, dt)
				y_0 = self.euler(v_y, y_0, dt)
				z_0 = self.euler(v_z, z_0, dt)
					
				msg.CmdPose.position.x = x_0
				msg.CmdPose.position.y = y_0
				msg.CmdPose.position.z = z_0
				
			elif msg.ControlType == 0: # publish velocity commands
				msg.CmdVel.linear.x = v_x
				msg.CmdVel.linear.y = v_y
				msg.CmdVel.linear.z = v_z
	
			#print (x_0,y_0,z_0)
			#print pos_msg.x, pos_msg.y, pos_msg.z
			self.pub.publish(msg)
	
			#rospy.spinOnce() # I dont need a spinOnce in rospy. rate.sleep has the same effect in rospy as ros::spinOnce in roscpp
	        	self.rate.sleep()
		

	def euler(self, f_x,x_prev,dt):
		x_cur = x_prev + f_x*dt
		return x_cur
	
	def saturate(self, Val,lim):
		if Val > lim:
			Val = lim
		elif Val < -lim:
			Val = -lim
		return Val

	def Odem_callback(self, data):
		self.pos_msg = data.NED
		self.twist_msg = data.NedVel
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.NED)
	
	def Cmd_callback(self, velcmd):
		self.cmd = velcmd


if __name__ == '__main__':
	control = SwarmController()
