#!/usr/bin/env python
import time
import math
import sys

import threading 
import rospy

import NodeDeps

from swarm_control.msg import gps_ins, command, actions
from geometry_msgs.msg import Pose, Twist

from swarm_control.srv import *

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from dronekit_sitl import start_default
from pymavlink import mavutil # Needed for command message definitions

class SoloInterface:
	def __init__(self, connection_string):
		self.PubRate = 10 # Publish Solo telemetry data at 10 hz

		# Create the ros node, publisher and subscriber
		rospy.init_node('Solo')
		self.ns = rospy.get_namespace()

		print(self.ns+": Connecting to vehicle on: %s" % connection_string)
		self.vehicle = connect(connection_string, wait_ready=False)

		self.vehicle.wait_ready('autopilot_version') # wait until Autopilot is ready
		# Get the home location from the autopilot
		while not self.vehicle.home_location:
			cmds = self.vehicle.commands
			cmds.download()
			cmds.wait_ready()
			if not self.vehicle.home_location:
				print(self.ns+": Waiting for home location ...")
			time.sleep(3)
		print(self.ns+": Got home location Lat=%f,Lon=%f,Alt=%f" % (self.vehicle.home_location.lat,self.vehicle.home_location.lon,self.vehicle.home_location.alt))
		
		# methods for frame conversion
		self.nav = NodeDeps.NavFrame()

		self.oldHome = self.vehicle.home_location
		self.Home = self.vehicle.home_location
		self.nav.setNavHome(self.oldHome.lat, self.oldHome.lon, self.oldHome.alt) # set reference origin to current solo Home

		# Create a publisher to publish solo telemetry 
		self.soloPose_Global = rospy.Publisher('Sensors/GPS_INS', gps_ins, queue_size=10, latch=True)

		#Call home service from GCS. Don't start anything unless GCS provides a Home location.
		rospy.wait_for_service('/GCS/Home')
		try:
			home_msg=rospy.ServiceProxy('/GCS/Home', HomeMsg)
			resp=home_msg()
			self.SoloSetHome(resp.Latitude, resp.Longitude, resp.Altitude) # Sets home to the common home for all solos
		except:
			print(self.ns+': Service Call to home failed')
			sys.exit(-1) # exit if GCS doesn't give a good home


		rospy.Subscriber('Commands/ControlPose', command, self.ReadSoloCmds) # Subscribe to the controller values
		rospy.Subscriber('Commands/Actions', actions, self.ReadSoloActions) # subscribe to get takeoff, land and arm commands

		time.sleep(1)
		# Start publishing
		self.PublishSoloTel()


	def PublishSoloTel(self):
		# set publish rate
		rate = rospy.Rate(self.PubRate)
		
		# Telemetry message
		Telemetry = gps_ins()

		print(self.ns+": Publishin tel data for solos")
		while not rospy.is_shutdown():

			# Get the GPS info from the vehicle
			Telemetry.GpsFix = self.vehicle.gps_0.fix_type
			Telemetry.NumSats = self.vehicle.gps_0.satellites_visible

			# Get latitude, longitude, altitude and relative altitude
			Telemetry.LLA.lat = self.vehicle.location.global_frame.lat
			Telemetry.LLA.lon = self.vehicle.location.global_frame.lon
			Telemetry.LLA.alt = self.vehicle.location.global_frame.alt
			Telemetry.LLA.relativealt = self.vehicle.location.global_relative_frame.alt

			'''
			# Get the NED information (if vehicle is disarmed send 0s)
			NOT_NED_VALUES = self.vehicle.location.local_frame.north == None
			if not NOT_NED_VALUES:
				Telemetry.NED.north = self.vehicle.location.local_frame.north
				Telemetry.NED.east = self.vehicle.location.local_frame.east
				Telemetry.NED.down = self.vehicle.location.local_frame.down
			else:
				Telemetry.NED.north = 0.0
				Telemetry.NED.east = 0.0
				Telemetry.NED.down = 0.0
			'''
			# convert LLA to NED 
			ned_out =self.nav.LLA2NED(self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt)
			Telemetry.NED.north = ned_out.north
			Telemetry.NED.east = ned_out.east
			Telemetry.NED.down = ned_out.down

			# Get the velocity information
			Telemetry.NedVel.x = self.vehicle.velocity[0]
			Telemetry.NedVel.y = self.vehicle.velocity[1]
			Telemetry.NedVel.z = self.vehicle.velocity[2]

			# Get the attitude information
			Telemetry.RPY.roll = self.vehicle.attitude.roll
			Telemetry.RPY.pitch = self.vehicle.attitude.pitch
			Telemetry.RPY.yaw = self.vehicle.attitude.yaw

			Telemetry.Battery = self.vehicle.battery.level
			Telemetry.FlightMode.data = str(self.vehicle.mode)

			#Telemetry.Arm = self.vehicle.armed
			Telemetry.Home.lat = self.nav.Home.lat #self.Home.lat
			Telemetry.Home.lon = self.nav.Home.lon #self.Home.lon
			Telemetry.Home.alt = self.nav.Home.alt #self.Home.alt

			self.soloPose_Global.publish(Telemetry)
			rate.sleep()


	def ReadSoloActions(self, Actions):
		# Get actions from ros topic and ensure safe arming, takeoff, landing
		if Actions.arm.data == True and self.vehicle.armed == False:
			self.SoloArm() # Arm

		if Actions.arm.data == False:
			self.SoloDisarm() # Disarm

		# If solo is still hovering when disarm command is called
		if Actions.arm.data == False and self.vehicle.location.global_relative_frame.alt > 3:
			self.SoloLand() # Land first
			time.sleep(5) # Sleep 5 secs
			self.SoloDisarm() # Disarm then

		if Actions.takeoff.data:
			if not Actions.land.data: # if land command is not active then takeoff
				self.SoloTakeOff(3)# takeoff and reach 3 m
			else:
				print("Landing Command Active! Set land flag to false to launch") # if both flags are set to true. FUTURE: Ensure flags are negation of each other

		if Actions.land.data:
			# If battery if reallyyyy low then land at location
			if self.vehicle.battery.level < 15:
				self.SoloLand()
			else:
				self.SoloRTHLand()


	def ReadSoloCmds(self, Cmd):
		#print Cmd.ControlType, self.vehicle.mode, Cmd.CmdVel.linear.x
		# if Mode is GUIDED only then publish commands. FUTURE: Make sure this check is safe!!!!
		if Cmd.ControlType == 0 and self.vehicle.mode == 'GUIDED': # Velocity Control
			velocity_x = Cmd.CmdVel.linear.x
			velocity_y = Cmd.CmdVel.linear.y
			velocity_z = Cmd.CmdVel.linear.z

			# Make sure altitude values are negative
			self.send_ned_velocity(velocity_x, velocity_y, velocity_z)

		elif Cmd.ControlType == 1 and self.vehicle.mode == 'GUIDED': #PositionControl
			north = Cmd.CmdPose.position.x
			east = Cmd.CmdPose.position.y
			down = Cmd.CmdPose.position.z

			#Make sure altitude values are negative
			lla_out=self.nav.NED2LLA(north, east, down)
			self.goto_position_target_global_int(lla_out) # DONT DO THIS. RELATIVE ALT NEEDS TO BE SENT FOR THIS METHOD

		else:
			#print(" Incorrect command control ")
			pass

	def SoloSetHome(self, newLat, newLon, newAlt):
		newHome_location = LocationGlobal(newLat, newLon, newAlt)
		self.vehicle.home_location = newHome_location
		self.Home = self.vehicle.home_location
		self.nav.setNavHome(self.Home.lat, self.Home.lon, self.Home.alt) # Update common home location
		
		print(self.ns+": New Home Location %s" % self.vehicle.home_location)


	def SoloArm(self):
		"""Arms vehicle. """

		print(self.ns+": Basic pre-arm checks")
		# Don't try to arm until autopilot is ready
		while not self.vehicle.is_armable:
			print(self.ns+": Waiting for vehicle to initialise...")
			time.sleep(1)

		print(self.ns+": Arming motors")

		# Copter should arm in GUIDED mode
		self.vehicle.mode = VehicleMode("GUIDED")
		self.vehicle.armed = True

		# Confirm vehicle armed before attempting to take off
		while not self.vehicle.armed:
			print(self.ns+": Waiting for arming...")
			time.sleep(1)
		print(self.ns+": Vehicle Armed")

	def SoloDisarm(self):
		print (self.ns+": Disarming vehicle")
		self.vehicle.armed = False

	def SoloTakeOff(self,aTargetAltitude):
		""" After Arming take off. """
		if self.vehicle.armed:
			print(self.ns+ ": Taking off!")
			self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

			while True:
				print(self.ns+": Altitude: ", self.vehicle.location.global_relative_frame.alt)
				# Break and return from function just below target altitude.
				if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
				    print(self.ns+": Reached target altitude")
				    break
				time.sleep(1)
		else:
			print(self.ns+": Vehicle not armed. Please send arm command ")


	def SoloRTHLand(self):
		print(self.ns+": Returning to original Home and Landing")
		self.vehicle.home_location = self.Home
		time.sleep(1)
		self.vehicle.mode = VehicleMode("RTL")
		self.vehicle.flush() # command clears are mission commands

	def SoloLand(self):
		print(self.ns+": Landing")
		self.vehicle.mode=VehicleMode("LAND")
		self.vehicle.flush() # clears all mission commands

	"""  #################### HELPER FUNCTIONS ############################## """

	def goto_position_target_local_ned(self, north, east, down):
		"""
		Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
		location in the North, East, Down frame.
		It is important to remember that in this frame, positive altitudes are entered as negative 
		"Down" values. So if down is "10", this will be 10 metres below the home altitude.
		Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
		ignored. For more information see: 
		http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned
		See the above link for information on the type_mask (0=enable, 1=ignore). 
		At time of writing, acceleration and yaw bits are ignored.
		"""
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
				0,       # time_boot_ms (not used)
				0, 0,    # target system, target component
				mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
				0b0000111111111000, # type_mask (only positions enabled)
				north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
				0, 0, 0, # x, y, z velocity in m/s  (not used)
				0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
				0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
		# send command to vehicle
		self.vehicle.send_mavlink(msg)


	def goto(self, dNorth, dEast):
		"""
		Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
		The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
		the target position. This allows it to be called with different position-setting commands. 
		By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
		The method reports the distance to target every two seconds.
		"""

		currentLocation = self.vehicle.location.global_relative_frame
		targetLocation = self.get_location_metres(currentLocation, dNorth, dEast)
		targetDistance = self.get_distance_metres(currentLocation, targetLocation)
		self.vehicle.simple_goto(targetLocation)

		#print "DEBUG: targetLocation: %s" % targetLocation
		#print "DEBUG: targetLocation: %s" % targetDistance

		while self.vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
			#print "DEBUG: mode: %s" % vehicle.mode.name
			remainingDistance=self.get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation)
			print("Distance to target: ", remainingDistance)
			if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
				print("Reached target")
				break;
			time.sleep(2)

	def get_location_metres(self,original_location, dNorth, dEast):
		"""
		Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
		specified `original_location`. The returned LocationGlobal has the same `alt` value
		as `original_location`.
		The function is useful when you want to move the vehicle around specifying locations relative to 
		the current vehicle position.
		The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
		For more information see:
		http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
		"""
		earth_radius = 6378137.0 #Radius of "spherical" earth
		#Coordinate offsets in radians
		dLat = dNorth/earth_radius
		dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

		#New position in decimal degrees
		newlat = original_location.lat + (dLat * 180/math.pi)
		newlon = original_location.lon + (dLon * 180/math.pi)
		if type(original_location) is LocationGlobal:
			targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
		elif type(original_location) is LocationGlobalRelative:
			targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
		else:
			raise Exception("Invalid Location object passed")

		return targetlocation;


	def get_distance_metres(self, aLocation1, aLocation2):
		"""
		Returns the ground distance in metres between two LocationGlobal objects.
		This method is an approximation, and will not be accurate over large distances and close to the 
		earth's poles. It comes from the ArduPilot test code:
		https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
		"""
		dlat = aLocation2.lat - aLocation1.lat
		dlong = aLocation2.lon - aLocation1.lon
		return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

	def send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
		"""
		Move vehicle in direction based on specified velocity vectors and
		for the specified duration.
		This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
		velocity components
		(http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).

		Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
		with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
		velocity persists until it is canceled. The code below should work on either version 
		(sending the message multiple times does not cause problems).

		See the above link for information on the type_mask (0=enable, 1=ignore). 
		At time of writing, acceleration and yaw bits are ignored.
		"""
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
				0,       # time_boot_ms (not used)
				0, 0,    # target system, target component
				mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
				0b0000111111000111, # type_mask (only speeds enabled)
				0, 0, 0, # x, y, z positions (not used)
				velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
				0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
				0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
		self.vehicle.send_mavlink(msg)


		# send command to vehicle on 1 Hz cycle (Raks: You will need this outside)
		#for x in range(0,duration):
		#vehicle.send_mavlink(msg)
		#time.sleep(1)


	def send_global_velocity(velocity_x, velocity_y, velocity_z):
    		"""
		Move vehicle in direction based on specified velocity vectors.
 		This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
		velocity components 
   		(http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
 		Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    		with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    		velocity persists until it is canceled. The code below should work on either version 
    		(sending the message multiple times does not cause problems).
    
    		See the above link for information on the type_mask (0=enable, 1=ignore). 
    		At time of writing, acceleration and yaw bits are ignored.
    		"""
		msg = vehicle.message_factory.set_position_target_global_int_encode(
        		0,       # time_boot_ms (not used)
        		0, 0,    # target system, target component
        		mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        		0b0000111111000111, # type_mask (only speeds enabled)
        		0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        		0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        		0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        		# altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        		velocity_x, # X velocity in NED frame in m/s
        		velocity_y, # Y velocity in NED frame in m/s
        		velocity_z, # Z velocity in NED frame in m/s
        		0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
		self.vehicle.send_mavlink(msg)



	def condition_yaw(heading, relative=False):
	    """
	    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
	    This method sets an absolute heading by default, but you can set the `relative` parameter
	    to `True` to set yaw relative to the current yaw heading.
	    By default the yaw of the vehicle will follow the direction of travel. After setting 
	    the yaw using this function there is no way to return to the default yaw "follow direction
	    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
	    For more information see: 
	    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
	    """
	    if relative:
			is_relative = 1 #yaw relative to direction of travel
	    else:
			is_relative = 0 #yaw is an absolute angle
	    # create the CONDITION_YAW command using command_long_encode()
	    msg = vehicle.message_factory.command_long_encode(
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
			0, #confirmation
			heading,    # param 1, yaw in degrees
			0,          # param 2, yaw speed deg/s
			1,          # param 3, direction -1 ccw, 1 cw
			is_relative, # param 4, relative offset 1, absolute angle 0
			0, 0, 0)    # param 5 ~ 7 not used
	    # send command to vehicle
	    self.vehicle.send_mavlink(msg)

	def goto_position_target_global_int(self, aLocation):
		"""
		Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.
		For more information see: https://pixhawk.ethz.ch/mavlink/#SET_POSITION_TARGET_GLOBAL_INT
		See the above link for information on the type_mask (0=enable, 1=ignore). 
		At time of writing, acceleration and yaw bits are ignored.
		"""
		msg = vehicle.message_factory.set_position_target_global_int_encode(
			0,       # time_boot_ms (not used)
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
			0b0000111111111000, # type_mask (only speeds enabled)
			aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
			aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
			aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
			0, # X velocity in NED frame in m/s
			0, # Y velocity in NED frame in m/s
			0, # Z velocity in NED frame in m/s
			0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
			0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
		# send command to vehicle
		vehicle.send_mavlink(msg)



if __name__ == '__main__':
	time.sleep(1)
	S= int(rospy.get_namespace()[-2])
	
	# Address on which vehicle is communicating
	MasterPort=sys.argv[1] #'tcp:127.0.0.1:5760'# sim
	RosInterfacePort='127.0.0.1:'+str(17000+S) # Loop back and pass to ros
	QGCInterfacePort='192.168.1.105:'+str(18000+S) #'192.168.0.155:'+str(18000+S) #IP of system running QGC

	t=threading.Thread(target=NodeDeps.launchMAVProxy, args=(MasterPort, RosInterfacePort, QGCInterfacePort,)) # Start mavproxy
	t.setDaemon(False) # thread SHOULD exit when main thread is killed
	t.start()

	time.sleep(5)
	connection_string='udpin:'+RosInterfacePort
	solo = SoloInterface(connection_string)

