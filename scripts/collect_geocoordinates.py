#!/usr/bin/env python

import rospy

from sensor_msgs.msg import NavSatFix

def main():

    rospy.init_node('geocoordinate_collector')
    print 'Waiting for GPS signal from satellite...'
    rospy.wait_for_message('android/fix', NavSatFix)
    print 'Waiting for filtered GPS signal.'
    rospy.wait_for_message('android/fix_filter', NavSatFix)

    print 'Start collecting geocoordinate...'
    geocoordinateSub = rospy.Subscriber('android/fix', NavSatFix, geocoordinate_cb)
    geocoordinateKfSub = rospy.Subscriber('android/fix_filter', NavSatFix, geocoordinate_kf_cb)
    rospy.spin()

def geocoordinate_cb(msg):
    with open('geocoordinates.txt','a') as f:
        print '%f %f' % (msg.latitude, msg.longitude)
        f.write('%f %f\n' % (msg.latitude, msg.longitude))
        f.close()

def geocoordinate_kf_cb(msg):
    with open('geocoordinates_filter.txt','a') as f:
        print '%f %f' % (msg.latitude, msg.longitude)
        f.write('%f %f\n' % (msg.latitude, msg.longitude))
        f.close()

if __name__ == '__main__':
    main()
