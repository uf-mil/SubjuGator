from txros import util
import numpy as np
from mil_ros_tools import rosmsg_to_numpy
from mil_misc_tools import FprintFactory

MISSION = "BUMP BUOYS"


class Buoy(object):
    def __init__(self):
        self.position = None
        self.bumped = False

    def set_bumped(self):
        self.bumped = True

    def update_position(self, pos):
        self.position = pos


class BumpBuoysMission(object):
    '''
    Mission to solve the recurring bump buoys RoboSub challenge.

    Designed to use the async features of txros to solve the mission in as little
    time as possible. One async function constantly pings the percption node for
    the latest pose, if available, for each color buoy. Another function continually
    checks if a new buoy has been found and performs moves to bump it. Another function
    runs search patterns (left, right, up, down) to attempt to gain more observations on
    buoys between bumping moves.
    '''
    START_FORWARD_METERS = 1.5  # Distance to move forward at start of mission so we're closer after aligning path marker
    AFTER_FORWARD_METERS = 4.0  # Distance to move forward from search start position to start finding path marke:
    ORDER = ['red', 'green', 'yellow']
    TIMEOUT_SECONDS = 30
    SEARCH_HEIGHT = 2.9  # Transdec
    SEARCH_STEPS = 10  # Number of moves to execute in search pattern
    SEARCH_PAUSE_SECONDS = 0.5
    Z_PATTERN_RADIUS = 0.4
    Y_PATTERN_RADIUS = 2.0
    BUMP_DISTANCE = 0.3  # Distance past buoy position to go to bump
    BACKUP_METERS = 3.0
    BLIND = False
    BUMP_SPEED = 0.5
    SEARCH_SPEED = 0.5

    def __init__(self, sub):
        self.search_origin = sub.move.to_height(self.SEARCH_HEIGHT).zero_roll_and_pitch().forward(self.START_FORWARD_METERS)
        self.sub = sub
        self.print_info = FprintFactory(title=MISSION).fprint
        self.print_bad = FprintFactory(title=MISSION, msg_color="red").fprint
        self.print_good = FprintFactory(title=MISSION, msg_color="green").fprint
        self.buoys = {'red': Buoy(), 'green': Buoy(), 'yellow': Buoy()}
        self.pattern_done = False
        self.generate_pattern()

    def generate_pattern(self):
        z = self.Z_PATTERN_RADIUS
        y = self.Y_PATTERN_RADIUS
        self.moves = []
        # Execute SEARCH_STEPS moves to strafe right and left Y_PATTERN_RADIUS meters
        for dy in np.linspace(0.0, -self.Y_PATTERN_RADIUS, num=self.SEARCH_STEPS/3):
            self.moves.append([0, dy, 0])
        for dy in np.linspace(-self.Y_PATTERN_RADIUS, self.Y_PATTERN_RADIUS, num=2*self.SEARCH_STEPS/3):
            self.moves.append([0, dy, 0])
        self.move_index = 0

    @util.cancellableInlineCallbacks
    def search(self):
        while True:
            info = 'FOUND: '
            for buoy in self.buoys:
                res = yield self.sub.vision_proxies.buoy.get_pose(target=buoy)
                if res.found:
                    self.buoys[buoy].update_position(rosmsg_to_numpy(res.pose.pose.position))
                if self.buoys[buoy].position is not None:
                    info += buoy + ' '
                yield self.sub.nh.sleep(0.5)  # Throttle service calls
            self.print_info(info)

    @util.cancellableInlineCallbacks
    def pattern(self):

        def err():
            self.print_info('Search pattern canceled')

        self.pattern_done = False
        for i, move in enumerate(self.moves[self.move_index:]):
            move = self.search_origin.relative(np.array(move)).go(blind=self.BLIND, speed=self.SEARCH_SPEED)
            move.addErrback(err)
            yield move
            yield self.sub.nh.sleep(self.SEARCH_PAUSE_SECONDS)
            self.move_index = i + 1
        self.print_bad('Pattern finished. Bumping any identifed buoys')
        self.pattern_done = True

    @util.cancellableInlineCallbacks
    def bump(self, buoy):
        self.print_info("BUMPING {}".format(buoy))
        buoy_position = self.buoys[buoy].position
        yield self.sub.move.go(blind=self.BLIND)  # Station hold
        yield self.sub.move.depth(-buoy_position[2]).go(blind=self.BLIND, speed=self.BUMP_SPEED)
        dist = np.linalg.norm(self.sub.pose.position - buoy_position)
        yield self.sub.move.look_at_without_pitching(buoy_position).forward(dist / 2.5).go(blind=self.BLIND, speed=self.BUMP_SPEED)
        self.print_info('Updating goal now that were closer')
        buoy_position = self.buoys[buoy].position
        yield self.sub.move.depth(-buoy_position[2]).go(blind=self.BLIND, speed=self.BUMP_SPEED)
        yield self.sub.move.look_at_without_pitching(buoy_position).set_position(buoy_position).forward(self.BUMP_DISTANCE).go(blind=self.BLIND, speed=self.BUMP_SPEED)
        self.print_good("{} BUMPED. Returning to start position".format(buoy))
        yield self.search_origin.go()

    def get_next_bump(self):
        '''
        Returns the color of the buoy that should be bumped now, or None.
        Tries to bump in the desired order, but if the search pattern is exhasted
        just goes for whichever have been found!
        '''
        for color in self.ORDER:
            if self.buoys[color].bumped:
                continue
            elif self.buoys[color].position is not None:
                return color
            elif self.pattern_done and self.buoys[color].position is not None:
                return color
            else:
                return None

    def done(self):
        bumped = sum(map(lambda b: 1 if self.buoys[b].bumped else 0, self.buoys))
        return bumped == len(self.ORDER)

    @util.cancellableInlineCallbacks
    def run(self):
        self.print_info('Moving forward to begin search')
        yield self.search_origin.go(speed=self.SEARCH_SPEED)
        self.print_info("Enabling Perception")
        self.sub.vision_proxies.buoy.start()
        pattern = self.pattern()
        self.do_search = True
        search = self.search()
        while not self.done():
            b = self.get_next_bump()
            if b is not None:
                pattern.cancel()
                yield self.bump(b)
                self.buoys[b].set_bumped()
                if not self.done():
                    pattern = self.pattern()
            elif self.pattern_done:
                break
            yield self.sub.nh.sleep(0.1)
        search.cancel()
        pattern.cancel()
        self.sub.vision_proxies.buoy.stop()
        self.print_good('Moving forward to see path marker...')
        yield self.search_origin.depth(0.65).go()
        yield self.search_origin.forward(self.AFTER_FORWARD_METERS).go(speed=self.SEARCH_SPEED)
        self.print_good('Done!')


@util.cancellableInlineCallbacks
def run(sub):
    mission = BumpBuoysMission(sub)
    yield mission.run()
