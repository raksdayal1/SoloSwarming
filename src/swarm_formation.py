#!/usr/bin/env python
import rospy
import NodeDeps
import time

from numpy import matrix, linalg, sort, nonzero, dot, asarray
from scipy.sparse.csgraph import csgraph_from_dense, laplacian, csgraph_to_dense

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from swarm_control.msg import State

from swarm_control.srv import StateMsg

class SwarmFormation:
	def Diff_callback(self, data):
		
		self.vec_x = data.deltax
		self.vec_y = data.deltay
		self.vec_z = data.deltaz
		
		self.dis2form_x = data.formx-data.posx
		self.dis2form_y = data.formy-data.posy
		self.dis2form_z = data.formz-data.posz
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.x)

	def VelSend(self):
		# Globals
		global vec_x
		global vec_y
		global vec_z
		global dis2form_x,dis2form_y,dis2form_z

		# initialization
		Out = NodeDeps.Init_Graph()
		Opt = NodeDeps.Swarm_Optimization(Out[1])
		c = Opt[0]
		B = Opt[1]
		F = Opt[2]

		# init Node
		rospy.init_node('Formation', anonymous=True)
		ns = rospy.get_namespace()

		# Publishers
		pub_vel = rospy.Publisher('command/cmd_vel', Twist, queue_size=10, latch=True)

		# Subscribes
		diff_topic = ns[0:-1]+ "/command/diff_vec"	
		rospy.Subscriber(diff_topic, State, self.Diff_callback)
		time.sleep(3)

		# Rate
		speed_rate = 50.0
		rate = rospy.Rate(speed_rate)
		node = int(ns[-2:-1])

		msg = Twist()
		while not rospy.is_shutdown():
			try:
				#print ns, c*B*F, Out[0][node-1,:], self.vec_x, self.vec_y
				#print ns + "UX = ",c*B*F*dot(Out[0][node-1,:], asarray(vec_x))		
				#print ns + "UY = ",c*B*F*dot(Out[0][node-1,:], asarray(vec_y))
				#print ns, self.dis2form_x, self.dis2form_y, self.dis2form_z
	
				ux_sig = -c*B*F*dot(Out[0][node-1,:], asarray(self.vec_x)) + 0.5*(self.dis2form_x)
				uy_sig = -c*B*F*dot(Out[0][node-1,:], asarray(self.vec_y)) + 0.5*(self.dis2form_y)
				uz_sig = -c*B*F*dot(Out[0][node-1,:], asarray(self.vec_z)) + 0.25*(self.dis2form_z)
				
				#print ux_sig
				msg.linear.x = ux_sig
				msg.linear.y = uy_sig
				msg.linear.z = uz_sig
			except:
				msg.linear.x = 0
				msg.linear.y = 0
				msg.linear.z = 0
			#print msg
			pub_vel.publish(msg)

			#rospy.spinOnce() # I dont need a spinOnce in rospy. rate.sleep has the same effect in rospy as ros::spinOnce in roscpp
			rate.sleep()



if __name__ == '__main__':
	swarmform = SwarmFormation()
	swarmform.VelSend()
