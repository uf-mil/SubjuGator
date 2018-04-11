from txros import util
from mil_passive_sonar.srv import  EstimatePingerPosition2D, EstimatePingerPosition2DRequest
from mil_misc_tools import text_effects
from twisted.internet import defer
import numpy as np


SEARCH_DEPTH = 1.0
UNDER_OCTAGON_DEPTH = 0.7
MIN_HEADINGS = 15
SEARCH_RADIUS = 1.5

@util.cancellableInlineCallbacks
def run(sub):
    fprint = text_effects.FprintFactory(title="OCTAGON SURFACE").fprint
    fprint('Creating service client')
    get_position = yield sub.nh.get_service_client('/passive_sonar/estimate_pinger_position_2d', EstimatePingerPosition2D)
    yield sub.nh.sleep(3.0)

    @util.cancellableInlineCallbacks
    def search_pattern():
        start = sub.move.depth(SEARCH_DEPTH)
        yield start.go()
        yield sub.nh.sleep(1.0)
        yield start.left(SEARCH_RADIUS).go()
        yield sub.nh.sleep(1.0)
        yield start.left(SEARCH_RADIUS).forward(SEARCH_RADIUS).go()
        yield sub.nh.sleep(1.0)
        yield start.right(SEARCH_RADIUS).forward(SEARCH_RADIUS).go()
        yield sub.nh.sleep(1.0)
        yield start.right(SEARCH_RADIUS).backward(SEARCH_RADIUS).go()

    fprint('Starting search')
    search = search_pattern()
    position = None
    while position is None:
       yield sub.nh.sleep(1.5)
       res = yield get_position(EstimatePingerPosition2DRequest())
       if not np.all(np.isfinite(np.array([res.x, res.y]))):
            fprint('position is NaN')
            continue
       fprint('get position with headings {}'.format(res.num_headings))
       if res.num_headings >= MIN_HEADINGS:
           print res
           position = np.array([res.x, res.y])
           break
    search.cancel()
    yield sub.move.forward(0.0)
    fprint('FOUND PINGER at {}, MOVING TO IT'.format(position))
    position = np.array([position[0], position[1], -UNDER_OCTAGON_DEPTH])
    fprint('Looking at pinger')
    yield sub.move.depth(UNDER_OCTAGON_DEPTH).look_at_without_pitching(position).go(speed=0.5)
    fprint('Going to pinger')
    print 'fucking positin', position
    yield sub.move.set_position(position).go(speed=0.5)
    fprint('SURFACING')
    yield sub.move.depth(0.0).go(speed=0.5)
    fprint('DONE')
