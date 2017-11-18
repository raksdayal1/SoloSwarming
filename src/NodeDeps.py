#!/usr/bin/env python
import rospy
import subprocess
import navpy

from dronekit import LocationGlobal, LocationGlobalRelative, LocationLocal

from os import devnull, getcwd

from numpy import matrix, linalg, sort, nonzero, array
from scipy.sparse.csgraph import csgraph_from_dense, laplacian, csgraph_to_dense

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist
from swarm_control.msg import State

from swarm_control.srv import StateMsg

#RESOURCE_PATH = '/home/ubuntu/rakshit/Projects/Swarming/swarm_ws/src/swarm_control'
RESOURCE_PATH = getcwd() + '/src/swarm_control'


def Init_Graph():
	# load the graph file
	file_name = RESOURCE_PATH + '/resource/graph_file.txt'
	f = open(file_name,'r')

	nodes = int(f.readline().split(':=')[1].strip('\n'))
	graph_type = f.readline().split(':=')[1].strip('\n')

	# open the adjacency matrix file
	adjmat_file_path = f.readline().split(':=')[1].strip('\n')
	adjmat_file = RESOURCE_PATH + '/resource/' + adjmat_file_path
	a = open(adjmat_file,'r')

	# read and store the adjacency matrix
	adj_matrix = matrix(a.read().replace('\n',';')[0:-1])
	sgraph_lap = laplacian(adj_matrix, normed=False)

	return adj_matrix, sgraph_lap, nodes

def Swarm_Optimization(sgraph_lap):
	# Currently no optimization is done. Calculations were done on a fixed system
	# parameters hardcoded
	# System x_dot = A*x+B*u
	A = 0
	B = 1

	# alpha =1, beta =1, mu = 2(calculated)
	P = 1.10 # from lyapunovs equation
	Q = 0.99 # from lyapunovs equation

	F = -B*(1.0/Q)

	eigen_values = sort(linalg.eigvals(sgraph_lap))
	real_eig = eigen_values.real
	c = (1.0/(real_eig[1])) + 0.01
	return c, B, F

def Get_Neighbors(sgraph, node_index, directed):
	# shift the index
	node_index -= 1
	if directed == True:
		neigh_list = list(nonzero(sgraph[:,node_index])[0] + 1)
	else:
		neigh_list = list(nonzero(sgraph[node_index,:])[0] + 1)
	return neigh_list

def launchMAVProxy(MasterPort, RosInterfacePort, QGCInterfacePort):
	# 17XXX is port used to send mav data to the ROS interface. Where XXX is System Id (e.q. 17001)
	# 18XXX is port used to send mav data to teh QGC interface.
	MasterId='--master='+str(MasterPort) # Recvd from vehicle or GroundUnit
	RosId='--out=udpbcast:'+str(RosInterfacePort) #Send to ROSINTERFACE
	QGCId='--out=udpbcast:'+str(QGCInterfacePort) #Send to QGCInterface

	FNULL = open(devnull, 'w')
	Task = subprocess.call(['mavproxy.py', MasterId, RosId, QGCId], stdout=FNULL, stderr=subprocess.STDOUT)


class NavFrame:
	def __init__(self, home_lat=0.0, home_lon=0.0, home_alt=0.0):
		self.Home = LocationGlobal(home_lat, home_lon, home_alt)

	def setNavHome(self, Latitude, Longitude, Altitude):
		self.Home = LocationGlobal(Latitude, Longitude, Altitude)

	def LLA2NED(self, curr_lat, curr_lon, curr_alt):
		NED = navpy.lla2ned(curr_lat, curr_lon, curr_alt, self.Home.lat, self.Home.lon, self.Home.alt, latlon_unit='deg', alt_unit='m', model='wgs84')
		return LocationLocal(NED[0], NED[1], NED[2])

	def NED2LLA(self, north, east, down):
		NED = array([north, east, down])
		LLA = ned2lla(NED, self.Home.lat, self.Home.lon, self.Home.alt, latlon_unit='deg', alt_unit='m', model='wgs84')
		return LocationGlobal(LLA[0], LLA[1], LLA[2])
