#!/usr/bin/env python
from __future__ import division

import txros
import txros.tf as tf
from twisted.internet import defer

from mil_misc_tools import text_effects

from sub8 import Searcher
from mil_ros_tools import rosmsg_to_numpy

import numpy as np

fprint = text_effects.FprintFactory(
    title="START_GATE", msg_color="cyan").fprint

# Distance before and after the gate in meters
FACTOR_DISTANCE_BEFORE = 1.5
FACTOR_DISTANCE_AFTER = 3

SPEED = 0.3


@txros.util.cancellableInlineCallbacks
def run(sub):
    yield sub.vision_proxies.start_gate.start()

    # Add search pattern if needed...
    fprint("Moving down a few meter")
    yield sub.move.depth(0.3).go()
    search_pattern = [sub.move.forward(10)]
    search = Searcher(
        sub,
        sub.vision_proxies.start_gate.get_pose,
        search_pattern)

    resp = None
    fprint('Searching...')
    resp = yield search.start_search(loop=False, timeout=50, spotings_req=2, speed=0.3)

    if resp is None or not resp.found:
        fprint("No gate found...", msg_color="red")
        defer.returnValue(None)

    position = rosmsg_to_numpy(resp.pose.pose.position)
    orientation = rosmsg_to_numpy(resp.pose.pose.orientation)

    point_before = 0
    point_after = 0

    distance = np.linalg.norm(sub.pose.position - position)
    while (distance > 1 and resp is not None and resp.found):

        fprint('Gate\'s position in map is: {}'.format(position))
        fprint('Gate\'s orientation in map is: {}'.format(orientation))
        # Get the normal vector, which is assumed to be the [1,0,0] unit vector
        normal = tf.transformations.quaternion_matrix(
            orientation).dot(np.array([1, 0, 0, 0]))[0:3]
        fprint('Computed normal vector: {}'.format(normal))

        # Computer points before and after the gate for the sub to go to
        point_before = position + FACTOR_DISTANCE_BEFORE * normal
        point_after = position - FACTOR_DISTANCE_AFTER * normal

        # go in front of gate
        fprint('Moving infront of gate {}'.format(point_before))
        yield sub.move.set_position(point_before).look_at(point_after).zero_roll_and_pitch().go(speed=SPEED)
        distance = np.linalg.norm(sub.pose.position - position)

        resp = yield sub.vision_proxies.start_gate.get_pose()

        position = rosmsg_to_numpy(resp.pose.pose.position)
        orientation = rosmsg_to_numpy(resp.pose.pose.orientation)

    fprint('YOLO! With style')
    yield style_points(sub)

        # go through the gate
    fprint('YOLO! Going through gate {}'.format(point_after))
    yield sub.move.set_position(point_after).zero_roll_and_pitch().go(speed=SPEED)

    yield sub.vision_proxies.start_gate.stop()

    defer.returnValue(True)

@txros.util.cancellableInlineCallbacks
def style_points(sub):
    yield sub.move.yaw_right(np.pi/2).go()
    yield sub.move.yaw_right(np.pi/2).go()
    yield sub.move.yaw_right(np.pi/2).go()
    yield sub.move.yaw_right(np.pi/2).go()
    yield sub.move.pitch_down(np.pi/2).go()
    yield sub.move.pitch_up(np.pi).go()
    yield sub.move.zero_roll_and_pitch().go()
