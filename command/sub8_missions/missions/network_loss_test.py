from txros import util
import genpy
from geometry_msgs.msg import WrenchStamped


@util.cancellableInlineCallbacks
def run(sub):
    '''
    Short mission to pubish a forward wrench, can be used to test
    network kill out of water (first kill the controller, call this mission
    from autonomous mission)
    '''
    wrench = WrenchStamped()
    wrench.header.frame_id = '/base_link'
    wrench.header.stamp = sub.nh.get_time()
    wrench.wrench.force.x = 5.0
    pub = sub.nh.advertise('/wrench', WrenchStamped)
    yield sub.nh.sleep(1.0)
    print "Going"
    end = sub.nh.get_time() + genpy.Duration(3.0)
    while sub.nh.get_time() < end:
        pub.publish(wrench)
        yield sub.nh.sleep(0.1)
