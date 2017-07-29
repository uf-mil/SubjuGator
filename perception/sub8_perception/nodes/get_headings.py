#!/usr/bin/env python
import rospy
from mil_passive_sonar.srv import GetPulseHeading, GetPulseHeadingRequest

get_pulse_heading = None  # Globals are bad and I should feel bad
INTERVAL = 3.0  # Time between heading requests in seconds

def call(timer_info):
    global get_pulse_heading
    try:
        res = get_pulse_heading.call(GetPulseHeadingRequest('bancroft'))
        rospy.loginfo('get heading is {}'.format(res.success))
    except Exception as e:
        print 'service error {}'.format(e)

if __name__ == '__main__':
    rospy.init_node('pinger_heading_doer')
    get_pulse_heading = rospy.ServiceProxy('/passive_sonar/get_pulse_heading', GetPulseHeading)
    rospy.Timer(rospy.Duration(INTERVAL), call)
    rospy.spin()


