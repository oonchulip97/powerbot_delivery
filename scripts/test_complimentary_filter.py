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
        self.num_geocoordinates = 5
        self.previous_lat_ = None
        self.previous_lng= None
        self.current_x = None
        self.current_y = None
        self.previous_x = None
        self.previous_y = None
        self.heading = None
        self.previous_delta_heading = 0

# GPS callback
def gps_cb(msg, state):

    gps_pub = rospy.Publisher('android/fix_cf', NavSatFix, queue_size=10)

    # Measure state
    state.measured_lat = msg.latitude
    state.measured_lng = msg.longitude
    state.measured_covariance = msg.position_covariance[0]

    # Predict state
    # In odom frame, x is direction robot is facing, y follows right-hand rule
    # In UTM frame, x is easting, y is northing
    odom = rospy.wait_for_message('rosaria/pose', Odometry)
    state.current_x = odom.pose.pose.position.x
    state.current_y = odom.pose.pose.position.y

    odom_delta_x = state.current_x - state.previous_x
    odom_delta_y = state.current_y - state.previous_y
    distance = sqrt(odom_delta_x ** 2 + odom_delta_y ** 2)
    theta = atan2(odom_delta_y, odom_delta_x)
    theta = (theta + 2 * pi) % (2 * pi) # in radians, positive is counter-clockwise from x-axis in odom frame from previous position to current position, interval is [0,+2pi)

    quaternion = (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    predicted_rotation = euler[2]
    predicted_rotation = (predicted_rotation + 2 * pi) % (2 * pi) # in radians, positive is counter-clockwise from x-axis in odom frame, interval is [0,2pi)
    predicted_heading = predicted_rotation + state.heading
    predicted_heading = predicted_heading % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,2pi)

    imu = rospy.wait_for_message('android/imu', Imu)
    quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    measured_heading = euler[2]
    measured_heading = (measured_heading + 2 * pi) % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)

    delta_heading = measured_heading - predicted_heading # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)
    heading = state.heading + state.previous_delta_heading
    heading = heading % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)

    magnetic_declination = 16.18 # degrees, positive is clockwise, interval is [-180,+180)
    magnetic_declination = (magnetic_declination + 360) % 360
    utm_odom_angle = theta + heading - radians(magnetic_declination) + (pi / 2)
    utm_odom_angle = (utm_odom_angle + 2 * pi) % (2 * pi) # radians, positive is counter-clockwise from x-axis of easting to current heading, interval is [0,360)
    utm_delta_x = distance * cos(utm_odom_angle)
    utm_delta_y = distance * sin(utm_odom_angle)

    previous_utm_x, previous_utm_y, zone_num, zone_let = utm.from_latlon(state.previous_lat, state.previous_lng)
    utm_x = previous_utm_x + utm_delta_x
    utm_y = previous_utm_y + utm_delta_y
    (predicted_lat, predicted_lng) = utm.to_latlon(utm_x, utm_y, zone_num, zone_let)

    # Complimentary filter
<<<<<<< HEAD
    min_covariance_threshold = 0 ** 2
    max_covariance_threshold = 10 ** 2
=======
    min_covariance_threshold = 5
    max_c_threshold = 10
>>>>>>> 7b9c74cfde212f9859ce0da0a9a54eb9e2b6483d

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
		gps_accuracy = 0.1 #(0,1)
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

    # Build message
    gps_msg = NavSatFix()
    gps_msg.header.seq = msg.header.seq
    gps_msg.header.stamp.secs = msg.header.stamp.secs
    gps_msg.header.stamp.nsecs = msg.header.stamp.nsecs
    gps_msg.header.frame_id = msg.header.frame_id
    gps_msg.status.status = msg.status.status
    gps_msg.status.service = msg.status.service
    gps_msg.latitude = estimated_lat
    gps_msg.longitude = estimated_lng
    gps_msg.altitude = msg.altitude
    gps_msg.position_covariance = msg.position_covariance
    gps_msg.position_covariance_type = msg.position_covariance_type
    gps_pub.publish(gps_msg)

    print 'Filtered GPS data published.'

def main():

    rospy.init_node('geocoordinate_complimentary_filter')

    state = State()
    lat_list = list()
    lng_list = list()
    odom_x_list = list()
    odom_y_list = list()
    heading_list = list()
    num_readings = 10

    # Initialize and average state
    print 'Waiting for GPS signal from satellite...'
    while len(lat_list) <= num_readings and len(lng_list) <= num_readings:
        location = rospy.wait_for_message('android/fix', NavSatFix)
        lat_list.append(location.latitude)
        lng_list.append(location.longitude)
    state.previous_lat = sum(lat_list)/len(lat_list)
    state.previous_lng = sum(lng_list)/len(lng_list)
    print 'GPS Signal Received.'

    print 'Waiting for Odometry...'
    while len(odom_x_list) <= num_readings and len(odom_y_list) <= num_readings:
        odom = rospy.wait_for_message('rosaria/pose', Odometry)
        odom_x_list.append(odom.pose.pose.position.x)
        odom_y_list.append(odom.pose.pose.position.y)
    state.previous_x = sum(odom_x_list)/len(odom_x_list)
    state.previous_y = sum(odom_y_list)/len(odom_y_list)
    print 'Odometry received.'

    print 'Waiting for IMU signal...'
    while len(heading_list) <= num_readings:
        imu = rospy.wait_for_message('android/imu', Imu)
        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2] # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [-pi,+pi)
        yaw = (yaw + 2 * pi) % (2 * pi) # in radians, 0 indicates northward direction, positive is counter-clockwise, interval is [0,+2pi)
        heading_list.append(yaw)
    state.heading = sum(heading_list)/len(heading_list)
    print 'IMU signal received.'

    gps_sub = rospy.Subscriber('android/fix', NavSatFix, gps_cb, state)

    rospy.spin()

if __name__ == '__main__':
    main()
