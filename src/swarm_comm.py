#!/usr/bin/env python
import rospy
import NodeDeps
import time

from numpy import matrix, linalg, sort, nonzero, zeros
from scipy.sparse.csgraph import csgraph_from_dense, laplacian, csgraph_to_dense

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from swarm_control.msg import State, gps_ins

from swarm_control.srv import *

class SwarmCommunication:
	def Odem_callback(self, data):
		# Globals
		self.pos_msg = data.NED
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)

	def FormConfig_callback(self, fdata): # change from Twist to Pose
		# Globals
		self.formconfig_msg = fdata.linear
		#rospy.loginfo(rospy.get_caller_id() + "I heard %s", fdata.linear)

	def handle_SendRadioMsg(self, req):
		return StateMsgResponse(self.pos_msg.north, self.pos_msg.east, self.pos_msg.down, self.formconfig_msg.x, self.formconfig_msg.y, self.formconfig_msg.z)

	def CommCenter(self):
		# Globals
		#global pos_msg, formconfig_msg

		# initialization
		Out = NodeDeps.Init_Graph()

		# init Node
		rospy.init_node('Communication', anonymous=True)

		ns = rospy.get_namespace()
		neighbors = NodeDeps.Get_Neighbors(Out[0], int(ns[-2]), True) # get neighbors sending data to uav
		
		# Publisher
	    	pub_dif = rospy.Publisher('command/diff_vec', State, queue_size=10, latch=True)

		# Subscribes
		pose_topic = ns[0:-1]+ '/Sensors/GPS_INS'
		rospy.Subscriber(pose_topic, gps_ins, self.Odem_callback)
	    	formconfig_topic = ns[0:-1]+ '/formconfig'
	    	rospy.Subscriber(formconfig_topic, Twist, self.FormConfig_callback) # change from Twist to Pose

		# Services
		radio_service = rospy.Service('radiomsg', StateMsg, self.handle_SendRadioMsg)
	
		time.sleep(10)
		
		# Rate
		speed_rate = 50.0
		rate = rospy.Rate(speed_rate)
		diff_vec_x = zeros((len(neighbors)+1,))#, dtype=np.int
		diff_vec_y = zeros((len(neighbors)+1,))#, dtype=np.int
		diff_vec_z = zeros((len(neighbors)+1,))

		msg = State()
		while not rospy.is_shutdown():
			#get service from all neighbors
			for neigh in  neighbors:
		    		service_to_call = ns[0:-2] + str(neigh) + '/radiomsg'	
				
				try:
					rospy.wait_for_service(service_to_call,timeout=1.0/speed_rate)
		    			neighbor_msg = rospy.ServiceProxy(service_to_call, StateMsg)
		    			resp = neighbor_msg()
					#print ns, str(neigh), resp.x, pos_msg.x, resp.form_x, formconfig_msg.x
					Ax = (resp.x - self.pos_msg.north) - (resp.form_x - self.formconfig_msg.x)
					Ay = (resp.y - self.pos_msg.east) - (resp.form_y - self.formconfig_msg.y)
					Az = (resp.z - self.pos_msg.down) - (resp.form_z - self.formconfig_msg.z)
					diff_vec_x[neigh-1] = Ax
					diff_vec_y[neigh-1] = Ay
					diff_vec_z[neigh-1] = Az
					
	
			    	except:
		    			#print ns+' '+service_to_call+ ' Service call failed '
					diff_vec_x[neigh-1] = 0
					diff_vec_y[neigh-1] = 0
					diff_vec_z[neigh-1] = 0
			
			try:		
				msg.posx = self.pos_msg.north
				msg.posy = self.pos_msg.east
				msg.posz = self.pos_msg.down
				msg.formx = self.formconfig_msg.x
				msg.formy = self.formconfig_msg.y
				msg.formz = self.formconfig_msg.z
			except:
				msg.posx = 0
				msg.posy = 0
				msg.posz = 0
				msg.formx = 0
				msg.formy = 0
				msg.formz = 0
					
			msg.deltax = diff_vec_x.tolist()
			msg.deltay = diff_vec_y.tolist()
			msg.deltaz = diff_vec_z.tolist()
			
			pub_dif.publish(msg)
			
			#rospy.spinOnce() # I dont need a spinOnce in rospy. rate.sleep has the same effect in rospy as ros::spinOnce in roscpp
			rate.sleep()


if __name__ == '__main__':
	swarmcomm = SwarmCommunication()
	swarmcomm.CommCenter()
