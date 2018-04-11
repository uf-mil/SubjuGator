from txros import util
from mil_passive_sonar.srv import  EstimatePingerPosition2D, EstimatePingerPosition2DRequest
from mil_misc_tools import text_effects
from twisted.internet import defer
import numpy as np

DEPTH = 0.7  # Depth to do search pattern and go to pinger
MIN_HEADINGS = 10
SEARCH_RADIUS = 1.5
SPEED = 0.5
GET_POSITION_SLEEP = 3.0
SEARCH_WAIT = 0.5


@util.cancellableInlineCallbacks
def run(sub):
    '''
    Mission to go to the 2D position of the pinger currently set
    to listen to, executing a little search pattern if it isn't found at first
    '''
    fprint = text_effects.FprintFactory(title="GO TO PINGER").fprint
    fprint('Creating service client')
    get_position = yield sub.nh.get_service_client('/passive_sonar/estimate_pinger_position_2d', EstimatePingerPosition2D)
    yield sub.nh.sleep(1.0)

    @util.cancellableInlineCallbacks
    def search_pattern():
        start = sub.move.depth(DEPTH).zero_roll_and_pitch()
        yield start.go()
        yield start.left(SEARCH_RADIUS).go(speed=SPEED, wait=SEARCH_WAIT)
        yield start.left(SEARCH_RADIUS).forward(SEARCH_RADIUS).go(speed=SPEED, wait=SEARCH_WAIT)
        yield start.right(SEARCH_RADIUS).forward(SEARCH_RADIUS).go(speed=SPEED, wait=SEARCH_WAIT)
        yield start.right(SEARCH_RADIUS).backward(SEARCH_RADIUS).go(speed=SPEED, wait=SEARCH_WAIT)

    fprint('Starting search')
    search = search_pattern()
    position = None
    while position is None:
       yield sub.nh.sleep(GET_POSITION_SLEEP)
       res = yield get_position(EstimatePingerPosition2DRequest())
       if not np.all(np.isfinite(np.array([res.x, res.y]))):
            fprint('position is NaN')
            continue
       fprint('got position with  {} headings'.format(res.num_headings))
       if res.num_headings >= MIN_HEADINGS:
           position = np.array([res.x, res.y])
           fprint('Found pinger at {}'.format(position))
           break
    search.cancel()
    fprint('FOUND PINGER at {}, MOVING TO IT'.format(position))
    position = np.array([position[0], position[1], -DEPTH])
    fprint('Looking at pinger')
    yield sub.move.depth(DEPTH).look_at_without_pitching(position).zero_roll_and_pitch().go(speed=SPEED)
    fprint('Going to pinger')
    yield sub.move.set_position(position).zero_roll_and_pitch().go(speed=SPEED)
    fprint('DONE')
