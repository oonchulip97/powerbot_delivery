#!/usr/bin/env python

import rospy
import tf
import utm

from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from math import pi, radians, sqrt, sin, cos, atan2

# Class to contain raw data of a state
class State(object):

	def __init__(self):
		self.measured_lat = None
		self.measured_lng = None
		self.measured_covariance = None
		self.lat_list = list()
		self.lng_list = list()
		self.num_geocoordinates = rospy.get_param('powerbot_delivery/num_geocoordinates')
		self.previous_lat_ = None
		self.previous_lng= None
		self.current_x = None
		self.current_y = None
		self.previous_x = None
		self.previous_y = None
		self.heading = None
		self.previous_delta_heading = 0
		self.location_received = False
		self.location = NavSatFix()
		self.odom = Odometry()

def gps_cb(msg, state):

	state.location = msg
	state.location_received = True

def odom_cb(msg, state):

	state.odom = msg

def update(state):

	# Measure state
	state.measured_lat = state.location.latitude
	state.measured_lng = state.location.longitude
	state.measured_covariance = state.location.position_covariance[0]

	# Predict state
	# In odom frame, x is direction robot is facing, y follows right-hand rule
	# In UTM frame, x is easting, y is northing
	state.current_x = state.odom.pose.pose.position.x
	state.current_y = state.odom.pose.pose.position.y

	odom_delta_x = state.current_x - state.previous_x
	odom_delta_y = state.current_y - state.previous_y
	distance = sqrt(odom_delta_x ** 2 + odom_delta_y ** 2)
	theta = atan2(odom_delta_y, odom_delta_x)
	theta = (theta + 2 * pi) % (2 * pi) # in radians, positive is counter-clockwise from x-axis in odom frame from previous position to current position, interval is [0,+2pi)

	quaternion = (state.odom.pose.pose.orientation.x, state.odom.pose.pose.orientation.y, state.odom.pose.pose.orientation.z, state.odom.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	predicted_rotation = euler[2]
	predicted_rotation = (predicted_rotation + 2 * pi) % (2 * pi) # in radians, positive is counter-clockwise from x-axis in odom frame, interval is [0,2pi)
	predicted_heading = predicted_rotation + state.heading
	predicted_heading = predicted_heading % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,2pi)

	imu_topic = rospy.get_param('powerbot_delivery/imu_topic_filter')
	imu = rospy.wait_for_message(imu_topic, Imu)
	quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	measured_heading = euler[2]
	measured_heading = (measured_heading + 2 * pi) % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)

	delta_heading = measured_heading - predicted_heading # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)
	heading = state.heading + state.previous_delta_heading
	heading = heading % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)

	magnetic_declination = rospy.get_param('powerbot_delivery/magnetic_declination') # degrees, varies with location, positive is clockwise, interval is [-180,+180)
	magnetic_declination = (magnetic_declination + 360) % 360
	yaw_offset = rospy.get_param('powerbot_delivery/yaw_offset') # degrees, positive is counter-clockwise, interval is [0,+360)
	utm_odom_angle = theta + heading - radians(magnetic_declination) + radians(yaw_offset) +(pi / 2)
	utm_odom_angle = (utm_odom_angle + 2 * pi) % (2 * pi) # radians, positive is counter-clockwise from x-axis of easting to current heading, interval is [0,360)
	utm_delta_x = distance * cos(utm_odom_angle)
	utm_delta_y = distance * sin(utm_odom_angle)

	previous_utm_x, previous_utm_y, zone_num, zone_let = utm.from_latlon(state.previous_lat, state.previous_lng)
	utm_x = previous_utm_x + utm_delta_x
	utm_y = previous_utm_y + utm_delta_y
	(predicted_lat, predicted_lng) = utm.to_latlon(utm_x, utm_y, zone_num, zone_let)

	# Complimentary filter
	min_covariance_threshold = rospy.get_param('powerbot_delivery/min_covariance_threshold')
	max_covariance_threshold = rospy.get_param('powerbot_delivery/max_covariance_threshold')

	# Moving average filter
	state.lat_list.append(state.measured_lat)
	state.lng_list.append(state.measured_lng)
	state.lat_list = state.lat_list[-state.num_geocoordinates:]
	state.lng_list = state.lng_list[-state.num_geocoordinates:]
	averaged_lat = sum(state.lat_list)/len(state.lng_list)
	averaged_lng = sum(state.lng_list)/len(state.lng_list)

	if state.measured_covariance >= max_covariance_threshold:
		estimated_lat = predicted_lat
		estimated_lng = predicted_lng

	elif state.measured_covariance <= min_covariance_threshold:
		estimated_lat = averaged_lat
		estimated_lng = averaged_lng

	else:
		gps_accuracy = (sqrt(max_covariance_threshold) - sqrt(state.measured_covariance)) / (sqrt(max_covariance_threshold) - sqrt(min_covariance_threshold)) #(0,1)
		estimated_lat = gps_accuracy * averaged_lat + (1 - gps_accuracy) * predicted_lat
		estimated_lng = gps_accuracy * averaged_lng + (1 - gps_accuracy) * predicted_lng

	# Update state
	state.lat_list = state.lat_list[:-1]
	state.lng_list = state.lng_list[:-1]
	state.lat_list.append(estimated_lat)
	state.lng_list.append(estimated_lng)
	state.previous_lat = estimated_lat
	state.previous_lng = estimated_lng
	state.previous_x = state.current_x
	state.previous_y = state.current_y
	state.previous_delta_heading = delta_heading

	return (estimated_lat, estimated_lng)

def intrapolate(state):

	# Intrapolate state
	# In odom frame, x is direction robot is facing, y follows right-hand rule
	# In UTM frame, x is easting, y is northing
	state.current_x = state.odom.pose.pose.position.x
	state.current_y = state.odom.pose.pose.position.y

	odom_delta_x = state.current_x - state.previous_x
	odom_delta_y = state.current_y - state.previous_y
	distance = sqrt(odom_delta_x ** 2 + odom_delta_y ** 2)
	theta = atan2(odom_delta_y, odom_delta_x)
	theta = (theta + 2 * pi) % (2 * pi) # in radians, positive is counter-clockwise from x-axis in odom frame from previous position to current position, interval is [0,+2pi)

	quaternion = (state.odom.pose.pose.orientation.x, state.odom.pose.pose.orientation.y, state.odom.pose.pose.orientation.z, state.odom.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	predicted_rotation = euler[2]
	predicted_rotation = (predicted_rotation + 2 * pi) % (2 * pi) # in radians, positive is counter-clockwise from x-axis in odom frame, interval is [0,2pi)
	predicted_heading = predicted_rotation + state.heading
	predicted_heading = predicted_heading % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,2pi)

	imu_topic = rospy.get_param('powerbot_delivery/imu_topic_filter')
	imu = rospy.wait_for_message(imu_topic, Imu)
	quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	measured_heading = euler[2]
	measured_heading = (measured_heading + 2 * pi) % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)

	delta_heading = measured_heading - predicted_heading # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)
	heading = state.heading + state.previous_delta_heading
	heading = heading % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)

	magnetic_declination = 26.18 # degrees, varies with location, positive is clockwise, interval is [-180,+180)
	magnetic_declination = (magnetic_declination + 360) % 360
	yaw_offset = rospy.get_param('powerbot_delivery/yaw_offset') # degrees, positive is counter-clockwise, interval is [0,+360)
	utm_odom_angle = theta + heading - radians(magnetic_declination) + radians(yaw_offset) +(pi / 2)
	utm_odom_angle = (utm_odom_angle + 2 * pi) % (2 * pi) # radians, positive is counter-clockwise from x-axis of easting to current heading, interval is [0,360)
	utm_delta_x = distance * cos(utm_odom_angle)
	utm_delta_y = distance * sin(utm_odom_angle)

	previous_utm_x, previous_utm_y, zone_num, zone_let = utm.from_latlon(state.previous_lat, state.previous_lng)
	utm_x = previous_utm_x + utm_delta_x
	utm_y = previous_utm_y + utm_delta_y
	(estimated_lat, estimated_lng) = utm.to_latlon(utm_x, utm_y, zone_num, zone_let)

	# Update state
	state.lat_list.append(estimated_lat)
	state.lng_list.append(estimated_lng)
	state.lat_list = state.lat_list[-state.num_geocoordinates:]
	state.lng_list = state.lng_list[-state.num_geocoordinates:]
	state.previous_lat = estimated_lat
	state.previous_lng = estimated_lng
	state.previous_x = state.current_x
	state.previous_y = state.current_y
	state.previous_delta_heading = delta_heading

	return (estimated_lat, estimated_lng)

def main():

	rospy.init_node('gps_filter')

	gps_pub = rospy.Publisher('android/fix_filter', NavSatFix, queue_size=10)

	state = State()
	lat_list = list()
	lng_list = list()
	num_geocoordinates_initial = rospy.get_param('powerbot_delivery/num_geocoordinates_initial')

	# Initialize state
	print 'Waiting for GPS signal from satellite for filter...'
	while len(lat_list) <= num_geocoordinates_initial and len(lng_list) <= num_geocoordinates_initial:
		gps_topic = rospy.get_param('powerbot_delivery/gps_topic_filter')
		location = rospy.wait_for_message(gps_topic, NavSatFix)
		lat_list.append(location.latitude)
		lng_list.append(location.longitude)
		print 'GPS signal received for filter.'
	state.previous_lat = sum(lat_list)/len(lat_list)
	state.previous_lng = sum(lng_list)/len(lng_list)
	print 'Sufficient GPS Signals received for averaging.'

	print 'Waiting for Odometry for filter...'
	odom_topic = rospy.get_param('powerbot_delivery/odom_topic_filter')
	odom = rospy.wait_for_message(odom_topic, Odometry)
	state.previous_x = odom.pose.pose.position.x
	state.previous_y = odom.pose.pose.position.y
	print 'Odometry received for filter.'

	print 'Waiting for IMU signal for filter...'
	imu_topic = rospy.get_param('powerbot_delivery/imu_topic_filter')
	imu = rospy.wait_for_message(imu_topic, Imu)
	quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	yaw = euler[2] # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [-pi,+pi)
	yaw = (yaw + 2 * pi) % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)
	state.heading = yaw
	print 'IMU signal received for filter.'

	# Receive inputs
	odom_sub = rospy.Subscriber(odom_topic, Odometry, odom_cb, state)
	gps_sub = rospy.Subscriber(gps_topic, NavSatFix, gps_cb, state)

	pub_freq = rospy.get_param('powerbot_delivery/pub_freq') # Hz
	rate = rospy.Rate(pub_freq)
	while not rospy.is_shutdown():

		# Update data
		if state.location_received:
			(estimated_lat, estimated_lng) = update(state)

		# Intrapolate data
		else:
			(estimated_lat, estimated_lng) = intrapolate(state)

		# Reset variable
		state.location_received = False

		# Build message
		gps_msg = NavSatFix()
		gps_msg.latitude = estimated_lat
		gps_msg.longitude = estimated_lng
		gps_pub.publish(gps_msg)

		rate.sleep()

if __name__ == '__main__':
	main()
