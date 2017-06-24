from txros import util
from twisted.internet import defer
import numpy as np
from mil_ros_tools import rosmsg_to_numpy
from mil_misc_tools import FprintFactory
import tf.transformations as trns

MISSION = "Collect Classify"


class Pipe(object):
    def __init__(self, color):
        self.color = color
        self.taken = False
        self.pose = None


class Oval(object):
    def __init__(self, color):
        self.pose = None
        self.full = False


class CollectClassifyMission(object):
    '''
    Class to carry out Collect and Classify mission in RoboSub2017.

    Assumes perception services are available to report found pipes in the tower
    and ovals on the table as 3D position and orientations and a perception service
    is available to report 3D position of the pinger at the center of the tower.

    A brief overview of the flow of the mission:
    * move in the direction of the pinger's heading until a pinger position estimate is available
    * Once pinger position is found, for each of the 4 tower pipers:
     * Circle around pinger while facing tower looking for a pipe
     * Once a pipe is found, grab pipe with gripper arm
     * backup and surface with pipe in gripper arm
     * submerge to depth above tower
     * If no ovals have been found:
       * Circle around octagon (large radius around pinger) until an oval on the table is found
     * Move above a previously found oval
     * Circle around oval table to find oval with same color as pipe in gripper
     * Once oval is found: move on top of oval and aligned with the longer axis
     * Open gripper arm to drop pipe onto table

    '''
    FREQ = 1300
    MIN_OBSERVATIONS = 10
    BLIND = True
    BASE_LINK_TO_ARM = np.array([0, 0, 0])  # Transformation from base_link to claw of arm mechanism
    COLORS = ['red', 'green', 'orange', 'blue']
    PIPE_SEARCH_HEIGHT = 0.6096  # Distance above ground to circle tower to find pipes, meters to DVL
    PIPE_SEARCH_RADIUS = 2.0  # Radius in meters around pinger to circle tower to find pipes
    OVAL_SEARCH_HEIGHT = 1.524  # Distance above ground to look for oval table, meters from ground to DVL
    OVAL_DROP_HEIGHT = 1.0  # Height to drop pipe onto table, meters from ground to DVL
    OCTOGON_RADIUS = 2.7  # Radius of octagon from rules, meters
    SAFE_SPEED = 0.3  # Proportion of sub's max speed to execute moves where collisions are possible (grabbing pipe)

    def __init__(self, sub):
        self.sub = sub
        self.pipes = list()
        self.pipes_found = list()
        self.ovals = list()
        self.ovals_found = list()
        for c in CollectClassifyMission.COLORS:  # Create pipe and oval objects for each color
            self.pipes.append(Pipe(c))
            self.ovals.append(Oval(c))

        self.print_bad = FprintFactory(title=MISSION, msg_color="red").fprint
        self.print_good = FprintFactory(title=MISSION, msg_coolr="green").fprint

    @staticmethod
    def vision_response_to_pose(res):
        '''
        Returns a tuple (position, orientation) representing the pose
        of a VisionRequest service response
        '''
        return (rosmsg_to_numpy(res.pose.pose.position), rosmsg_to_numpy(res.pose.pose.orientation))

    @util.cancellableInlineCallbacks
    def go_to_pinger(self):
        '''
        Move towards pinger at center of tower. Incrementally moves in direction
        of pinger heading if available. Once position estimate is available, move
        a few meters backwards from pinger.
        '''
        yield self.sub.passive_sonar.set_frequency(self.FREQ)
        yield self.sub.passive_sonar.reset()
        self.pinger_position = None
        torward_move = None

        @util.cancellableInlineCallbacks
        def get_pinger_pose():
            while self.pinger_position is None:
                res = yield self.sub.passive_sonar.estimate_position()
                if res.num_headings >= self.MIN_OBSERVATIONS:
                    assert res.header.frame_id in ('/map', 'map')
                    self.pinger_position = np.array([res.x, res.y, res.z])
                yield self.sub.nh.sleep(0.5)

        while self.pinger_position is None:
            heading = yield self.sub.passive_sonar.get_heading()
            if heading.success:
                hydrophones_to_baselink = yield self.sub._tf_listener.get_transform('/map', heading.header.frame_id,
                                                                                    heading.header.stamp)
                self.pinger_heading = hydrophones_to_baselink.transform_vector(np.array([heading.x, heading.y,
                                                                                         heading.z]))
                self.pinger_heading[2] = 0.0
                self.pinger_heading = self.pinger_heading / np.linalg.norm(self.pinger_heading)
                goal = self.sub.pose.position + self.pinger_heading

                torward_move = self.sub.move.look_at(goal).forward(1).go(blind=self.BLIND)
                if self.pinger_position is None:
                    yield torward_move
                torward_move = None
            else:
                yield self.sub.nh.sleep(0.5)

        print 'MOVING TO PONGO', self.pinger_position
        yield self.sub.move.look_at(self.pinger_position).set_position(
            self.pinger_position).backward(3.0).go(blind=self.BLIND)

    @util.cancellableInlineCallbacks
    def surface(self):
        '''
        Surface sub at current position, wait a few seconds,
        then submerge again.
        '''
        yield self.sub.move.depth(0).go(check_feasibility=False)
        yield self.sub.nh.sleep(2.0)
        yield self.sub.move.to_height(CollectClassifyMission.OVAL_SEARCH_HEIGHT).go()

    @util.cancellableInlineCallbacks
    def find_pipe(self):
        '''
        Circle around pinger, looking for pipes in the tower. If one is found,
        return the Pipe object. Otherwise, return None.
        '''
        yield self.sub.vision_proxies.tower_pipes.start()
        circle_done = False

        @util.cancellableInlineCallbacks
        def circle_pinger():
            yield self.sub.move.to_height(CollectClassifyMission.PIPE_SEARCH_HEIGHT).circle(
                self.pinger_position, CollectClassifyMission.PIPE_SEARCH_RADIUS)
            self.circle_done = True
        search = circle_pinger()
        while not circle_done:
            res = self.get_pipe()
            if res is not None:
                # TODO: check frame, transform to map if needed
                circle_done = True
                search.cancel()
                yield self.sub.vision_proxies.tower_pipes.stop()
                defer.returnValue(res)
        yield self.sub.vision_proxies.tower_pipes.stop()
        defer.returnValue(None)

    @util.cancellableInlineCallbacks
    def pickup_pipe(self, pose):
        '''
        Execute moves to grab a pipe (given its pose)
        '''
        # Move behind
        move = self.sub.move.set_position(pose[0]).set_orientation(pose[1]).yaw_left(
            np.pi / 2.0).zero_roll_and_pitch()
        # Adjust for gripper tf
        move = move.relative(CollectClassifyMission.BASE_LINK_TO_ARM)
        # Rotate 180 if we're not facing pinger
        m = trns.quaternion_matrix(self.sub.move._pose.orientation).dot(np.array([0.1, 0, 0]))
        if np.linalg.norm(m - self.pinger_position) > np.linalg.norm(move._pose.position - self.pinger_position):
            move = move.yaw_left(np.pi)

        move = move.backward(1.5)
        yield move.go()

        # Open arm
        yield self.sub.actuators.gripper_open()

        # Move forward
        move = move.forward(1.0)
        yield move.go(speed=CollectClassifyMission.SAFE_SPEED)
        yield self.sub.acutators.gripper_close()  # Close arm around pipe

        yield self.sub.move.up(2.0).go(speed=CollectClassifyMission.SAFE_SPEED)  # move up to lift pipe

    @util.cancellableInlineCallbacks
    def get_oval(self):
        '''
        Asks oval perception for the pose of each unfound oval. Returns the Oval
        object of the first one found, or None if none are found.
        '''
        for o in self.ovals:
            res = self.sub.vision_proxies.table_ovals.get_pose(target=o.color)
            if res.found:
                # TODO: check frame, convert if needed
                o.pose = self.vision_response_to_pose(res)
                self.ovals_found.append(o)
                self.ovals.remove(o)
                defer.returnValue(o)
        defer.returnValue(None)

    @util.cancellableInlineCallbacks
    def get_pipe(self):
        '''
        Asks pipe perception for the pose of each unfound pipe. Returns the Pipe
        object of the first one found, or None if none are found.
        '''
        for p in self.pipes:
            res = self.sub.vision_proxies.tower_pipes.get_pose(target=p.color)
            if res.found:
                # TODO: check frame, convert if needed
                p.pose = self.vision_response_to_pose(res)
                self.pipes_found.append(p)
                self.ovals.remove(p)
                defer.returnValue(p)
        defer.returnValue(None)

    @util.cancellableInlineCallbacks
    def find_first_oval(self):
        '''
        Circle at octagon radius around pinger to find the oval table.
        If an oval has already been found or if one is found during search pattern,
        return True. Otherwise, return False.
        '''
        if len(self.ovals_found) > 0:
            defer.returnValue(True)
        circle_done = False

        @util.cancellableInlineCallbacks
        def circle():
            yield self.sub.move.to_height(CollectClassifyMission.OVAL_SEARCH_HEIGHT).circle(
                self.pinger_position, CollectClassifyMission.OCTOGON_RADIUS)
            circle_done = True  # noqa

        search = circle()
        while not circle_done:
            res = yield self.get_oval()
            if res is not None:
                search.cancel()
                defer.returnValue(True)
        defer.returnValue(False)

    @util.cancellableInlineCallbacks
    def find_oval(self, color):
        '''
        Find the pose of the oval with specified color. If no ovals have been found,
        circle around pinger at large radius to find the table. Once found,
        circle around a previously found oval to find the correct oval.

        If correct color oval not found after search pattern, return pose of a random
        found oval. If still no ovals are found, return None.
        '''
        # Enable oval perception
        yield self.sub.vision_proxies.table_ovals.start()

        # Go to safe depth so we don't hit tower
        yield self.sub.move.to_height(CollectClassifyMission.OVAL_SEARCH_HEIGHT).go()

        # Circle outside octagon to find table if no ovals found yet
        table_found = yield self.find_first_oval()
        if not table_found:
            yield self.sub.vision_proxies.table_ovals.stop()
            defer.returnValue(None)

        # Exit now if correct color is found
        for o in self.ovals_found:
            if o.color == color:
                yield self.sub.vision_proxies.table_ovals.stop()
                defer.returnValue(o)

        search_done = False

        @util.cancellableInlineCallbacks
        def search_table():
            assert len(self.ovals_found) != 0
            start = self.ovals_found[np.random.randint(0, len(self.oval_found))].pose[0]
            yield self.sub.move.to_height(CollectClassifyMission.OVAL_SEARCH_HEIGHT).circle(
                start, CollectClassifyMission.TABLE_RADIUS)
            search_done = True  # noqa

        while not search_done:
            o = yield self.get_oval()
            if o is not None and o.color == color:
                yield self.sub.vision_proxies.table_ovals.stop()
                defer.returnValue(o)
        yield self.sub.vision_proxies.table_ovals.stop()

        # Return a random found oval if we can't find the one we want
        if len(self.ovals_found) == 0:
            defer.returnValue(None)
        o = self.ovals_found[np.random.randint(0, len(self.ovals_found))]
        self.print_bad('{} oval not found. Dropping pipe onto {} oval'.format(o.color))
        defer.returnValue(o)

    @util.cancellableInlineCallbacks
    def drop_pipe(self, pose):
        '''
        Given the pose of a circle on the table, execute moves
        to drop pipe onto circle.
        '''
        # Move at safe depth above oval
        yield self.sub.move.set_position(pose[0] + CollectClassifyMission.BASE_LINK_TO_ARM).to_height(
            CollectClassifyMission.OVAL_SEARCH_HEIGHT).set_orientation(pose[1]).zero_roll_and_pitch().go()

        # Descend to right above oval
        yield self.sub.move.to_height(CollectClassifyMission.OVAL_DROP_HEIGHT).go(
            speed=CollectClassifyMission.SAFE_SPEED)

        # Release grabber
        yield self.sub.actuators.gripper_open()

    @util.cancellableInlineCallbacks
    def score_pipe(self):
        '''
        1) Circle pinger looking for a remaining pipe
        2) Grab pipe, backup
        3) Surface with pipe without leaving octagon
        4) Submerge and search for corresponding oval
        5) release pipe onto correct oval or a random one if not found
        '''
        pipe = yield self.find_pipe()  # Run pattern to get a remaining pipe's pose
        if pipe is None:
            self.print_bad('Could not find a pipe')
            defer.returnValue(False)
        self.print_good('Picking up {} pipe'.format(pipe.color))
        yield self.grab_pipe(pipe.pose)  # Grab pipe with gripper arm)
        # Set pipe object to taken
        try:
            pipe_index = self.pipes_found.index(pipe)
            self.pipes_found[pipe_index].taken = True
        except ValueError:
            self.print_bad("Could not set pipe taken, this shouldn't happen.")
        self.print_good('Surfacing with {} pipe'.format(pipe.color))
        yield self.surface()
        self.print_good('Searching for {} oval'.format(pipe.color))
        oval = yield self.find_oval(pipe.color)
        if oval is None:
            self.print_bad('Could not find {} oval'.format(pipe.color))
            defer.returnValue(False)
        self.print_good('Dropping {} pipe onto oval'.format(oval.color))
        yield self.drop_pipe(oval.pose)
        # Set oval object to full (with pipe on it)
        try:
            oval_index = self.ovals_found.index(oval)
            self.ovals_found[oval_index].full = True
        except ValueError:
            self.print_bad("Could not set oval full, this shouldn't happen.")
        self.print_good('{} collected and classified'.format(oval.color))
        defer.returnValue(True)

    @util.cancellableInlineCallbacks
    def run(self):
        for i in xrange(len(CollectClassifyMission.colors)):
            self.print_good('Collecting and classifying pipe #{}'.format(i))
            success = yield self.score_pipe()
            if success:
                self.print_good('Pipe #{} collected and classified!'.format(i))
            else:
                self.print_bad('Failed to collect and classify Pipe #{}'.format(i))
        pass


@util.cancellableInlineCallbacks
def run(sub):
    mission = CollectClassifyMission(sub)
    yield mission.run()
