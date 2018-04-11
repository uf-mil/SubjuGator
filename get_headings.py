#!/usr/bin/env python
import rospy
from mil_passive_sonar.srv import GetPulseHeading, GetPulseHeadingRequest

get_pulse_heading = None
TIME=3.0

def call(timer_info):
    global get_pulse_heading
    print 'timer'
    try:
        res = get_pulse_heading.call(GetPulseHeadingRequest('bancroft'))
        if res is None:
            print 'heading is none'
        else:
            print 'get heading is {}'.format(res.success)
    except Exception as e:
        print 'service error {}'.format(e)

if __name__ == '__main__':
    print 'main'
    rospy.init_node('pinger_heading_doer')
    get_pulse_heading = rospy.ServiceProxy('/passive_sonar/get_pulse_heading', GetPulseHeading)
    print 'starting timer'
    rospy.Timer(rospy.Duration(TIME), call)
    print 'spinnig'
    rospy.spin()


