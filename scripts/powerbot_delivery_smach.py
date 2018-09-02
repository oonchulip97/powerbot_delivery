#!/usr/bin/env python

import rospy
import smach
import smach_ros
import subprocess
import tf
import json
import lz4
import os
import threading

from smach import CBState, Concurrence
from smach_ros import SimpleActionState
from math import sqrt, sin, cos, atan2, radians, degrees

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import NavSatFix, Imu

# Define class for geographic coordinate system
class GeoCoordinate:
	def __init__(self, lat, lng):
		self.lat = lat
		self.lng = lng

# Define state GET_INITIAL_LOCATION
class GetInitialLocation(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'])

	def execute(self, userdata):

		rospy.loginfo('Executing state GET_INITIAL_LOCATION')

		# Retrieve current location
		print 'Waiting for GPS signal from satellite for SMACH...'
		while not rospy.is_shutdown():
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'
			try:
				gps_topic = rospy.get_param('powerbot_delivery/gps_topic_smach')
				location = rospy.wait_for_message(gps_topic, NavSatFix, timeout = 1)
				break
			except:
				pass
		rospy.set_param('start_ros/lat',location.latitude)
		rospy.set_param('start_ros/lng',location.longitude)
		print 'GPS Signal Received for SMACH.'

		print 'Initial location obtained for SMACH.'
		return 'succeeded'

# Define state SELECTION
class Selection(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'], input_keys=['waypoints'], output_keys=['waypoint','waypoints'])

	def execute(self, userdata):

		rospy.loginfo('Executing state SELECTION')

		# Reset waypoint's index and waypoints as new route is being selected
		userdata.waypoint = 0
		userdata.waypoints = list()
		if (rospy.has_param('waypoints_ros')):
			rospy.delete_param('waypoints_ros')

		# Open or reload web console to select destination
		reload_path = rospy.get_param('powerbot_delivery/reload_path')
		map_path = rospy.get_param('powerbot_delivery/map_path')
		map_url = rospy.get_param('powerbot_delivery/map_url')
		current_url = ''

		# Check whether Firefox is opened or not
		if (os.path.isfile(reload_path) and (subprocess.call('ps -A | grep firefox', shell = True) == 0)):
			f = open(reload_path,'r')
			magic = f.read(8) # First 8 bytes in recovery.jsonlz4 are part of mozillas custom file format
			jdata = json.loads(lz4.decompress(f.read()).decode('utf-8'))
			f.close()

			# Find current active tab
			number_of_selected_tab = jdata["windows"][0]["selected"]
			tab_number = 1
			for win in jdata.get("windows"):
				for tab in win.get("tabs"):
					if number_of_selected_tab == tab_number:
						tab_index = tab.get("index") - 1
						current_url = tab.get("entries")[tab_index].get("url")
					tab_number = tab_number + 1

		# Check whether current active tab is map.html or not
		if (current_url == map_url):
			subprocess.call('xvkbd -window Firefox -text "\Cr" &', shell = True)
		else:
			open_cmd = 'firefox %s &' % map_path
			subprocess.call(open_cmd, shell = True)

		while not rospy.is_shutdown():
			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'

			if (rospy.has_param('waypoints_ros')):
				waypoints = rospy.get_param('waypoints_ros')
				for step in waypoints:
					userdata.waypoints.append(GeoCoordinate(step['lat'],step['lng']))
				return 'succeeded'

# Define state NAVIGATE
class Navigate(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'], input_keys=['waypoint','waypoints'])

	def execute(self, userdata):

		rospy.loginfo('Executing state SEND_GOAL')

		waypoint_pub = rospy.Publisher('waypoint', Pose, queue_size=10)
		goal_tolerance = rospy.get_param('goal_tolerance') # How close current position to goal should be to be considered arrived

		rate = rospy.Rate(10)
		while not rospy.is_shutdown():

			if self.preempt_requested():
				self.service_preempt()
				return 'preempted'

			# Retrieve current location
			print 'Waiting for GPS signal from satellite for SMACH...'
			while not rospy.is_shutdown():
				if self.preempt_requested():
					self.service_preempt()
					return 'preempted'
				try:
					gps_topic = rospy.get_param('powerbot_delivery/gps_topic_smach')
					location = rospy.wait_for_message(gps_topic, NavSatFix, timeout = 1)
					break
				except:
					pass
			currentPosition = GeoCoordinate(location.latitude,location.longitude)
			print 'GPS Signal Received for SMACH.'

			# Retrieve heading
			print 'Waiting for IMU signal for SMACH...'
			while not rospy.is_shutdown():
				if self.preempt_requested():
					self.service_preempt()
					return 'preempted'
				try:
					imu_topic = rospy.get_param('powerbot_delivery/imu_topic_smach')
					imu = rospy.wait_for_message('imu_topic', Imu, timeout = 1)
					break
				except:
					pass
			quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			heading = degrees(euler[2]) # in degrees, 0 indicates northward direction, positive is counter-clockwise, interval is [+180 and -180]
			print 'IMU signal received for SMACH.'

			lat1 = currentPosition.lat
			lng1 = currentPosition.lng
			lat2 = userdata.waypoints[userdata.waypoint].lat
			lng2 = userdata.waypoints[userdata.waypoint].lng

			distance = haversine(lat1,lng1,lat2,lng2)
			bearing = angle(lat1,lng1,lat2,lng2) # degrees
			direction = (bearing - heading) % 360 # degrees

			# Check whether robot arrived at waypoint or not
			if distance <= goal_tolerance:
				return 'succeeded'

			goal = Pose()

			goal.position.x = distance * cos(radians(direction))
			goal.position.y = distance * sin(radians(direction))
			goal.position.z = 0
			goal.orientation.x = 0
			goal.orientation.y = 0
			goal.orientation.z = 0
			goal.orientation.w = 1

			waypoint_pub.publish(goal)

			rate.sleep()

		return 'aborted'

# Define state CHECK_WAYPOINT
class CheckWaypoint(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded','continue','preempted','aborted'], input_keys=['waypoints','waypoint'])

	def execute(self, userdata):

		rospy.loginfo('Executing state CHECK_WAYPOINT')

		if waypoint == (len(waypoints)-1):
			print 'Navigation is successful.'
			return 'succeeded'
		else:
			print 'Continuing to next waypoint.'
			return 'continue'

# Define function which calculate distance using the Haversine Formula
def haversine(lat1, lng1, lat2, lng2):

	R = 6371000  # Radius of Earth in meters
	phi_1 = radians(lat1)
	phi_2 = radians(lat2)
	delta_lat = radians(lat2 - lat1)
	delta_lng = radians(lng2 - lng1)

	a = sin(delta_lat / 2.0) ** 2 + cos(phi_1) * cos(phi_2) * sin(delta_lng / 2.0) ** 2
	c = 2 * atan2(sqrt(a), sqrt(1 - a))
	distance = R * c

	# Output distance in meters
	return distance

# Define function which calculate angle between two points
def angle(lat1, lng1, lat2, lng2):

	phi_1 = radians(lat1)
	phi_2 = radians(lat2)
	delta_lng = radians(lng2 - lng1)

	y = sin(delta_lng) * cos(phi_2)
	x = cos(phi_1) * sin(phi_2) - sin(phi_1) * cos(phi_2) * cos(delta_lng)

	brng = atan2(y, x)
	brng = degrees(brng)
	brng = (brng + 360) % 360
	brng = 360 - brng

	# Output bearing in degrees
	# 0 degrees indicates northward heading
	# Count bearing in counter-clockwise direction
	return brng

@smach.cb_interface(input_keys=['waypoint'],output_keys=['waypoint'],outcomes=['done'])
# Function for prepping next waypoint.
def nextWaypoint(userdata):

	rospy.loginfo('Executing state NEXT_WAYPOINT')
	userdata.waypoint += 1
	return 'done'

def main():

	rospy.init_node('powerbot_delivery_state_machine')

	# Create the top level SMACH state machine
	sm_delivery = smach.StateMachine(outcomes=['finished','preempted','aborted'])

	sm_delivery.userdata.waypoint = 0

	# Open the top level container
	with sm_delivery:

		# Add states to the top level container
		smach.StateMachine.add('GET_INITIAL_LOCATION', GetInitialLocation(), transitions={'succeeded':'SELECTION','preempted':'preempted','aborted':'aborted'})

		smach.StateMachine.add('SELECTION', Selection(), transitions={'succeeded':'NAVIGATE','preempted':'preempted','aborted':'aborted'}, remapping={'waypoint':'waypoint','waypoints':'waypoints'})

		smach.StateMachine.add('NAVIGATE', Navigate(), transitions={'succeeded':'CHECK_WAYPOINT','preempted':'preempted','aborted': 'aborted'}, remapping={'waypoints':'waypoints','waypoint':'waypoint'})

		smach.StateMachine.add('CHECK_WAYPOINT', CheckWaypoint(), transitions={'succeeded':'GET_INITIAL_LOCATION','continue':'NEXT_WAYPOINT','preempted':'preempted','aborted':'aborted'}, remapping={'waypoints':'waypoints','waypoint':'waypoint'})

		smach.StateMachine.add('NEXT_WAYPOINT', CBState(nextWaypoint), {'done':'NAVIGATE'})

	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('powerbot_delivery_server', sm_delivery, '/sm_delivery')
	sis.start()

	# Set preemption handler
	smach_ros.set_preempt_handler(sm_delivery)

	# Create a thread to execute the smach container
	smach_thread = threading.Thread(target = sm_delivery.execute)
	smach_thread.start()

	# Wait for ctrl-c
	rospy.spin()

	# Request the container to preempt
	sm_delivery.request_preempt()

	# Block until everything is preempted
	smach_thread.join()

	# Stop introspection
	sis.stop()

if __name__ == '__main__':
	main()
