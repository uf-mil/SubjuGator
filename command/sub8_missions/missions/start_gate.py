#!/usr/bin/env python
from __future__ import division

import txros
import txros.tf as tf
from twisted.internet import defer

from mil_misc_tools import text_effects

from sub8 import Searcher
from mil_ros_tools import rosmsg_to_numpy

from geometry_msgs.msg import Point, PoseStamped

import numpy as np

fprint = text_effects.FprintFactory(
    title="START_GATE", msg_color="cyan").fprint

# Distance before and after the gate in meters
FACTOR_DISTANCE_BEFORE = 1.5
FACTOR_DISTANCE_AFTER = 3

SPEED = 0.3

GATE_DISTANCE = 12
TRIES = 10


@txros.util.cancellableInlineCallbacks
def run(sub):
    # yield sub.vision_proxies.start_gate.start()
    yield sub.nh.sleep(1)
    subscriber = sub.nh.subscribe('gate_point', PoseStamped)
    msg = yield subscriber.get_next_message()
    print msg
    point = rosmsg_to_numpy(msg.pose.position)
    # point = np.array([15, -10, 2]) - np.array([13, 24, 0])
    yield sub.move.depth(.7).go()
    yield sub.move.look_at(point).zero_roll_and_pitch().go()
    yield sub.move.forward(13).zero_roll_and_pitch().go()

    defer.returnValue(None)

    # Add search pattern if needed...
    fprint("Moving down a few meter")
    yield sub.move.depth(0.3).zero_roll_and_pitch().go()
    search_pattern = [sub.move.forward(12)]
    search = Searcher(
        sub,
        sub.vision_proxies.start_gate.get_pose,
        search_pattern)

    resp = None
    fprint('Searching...')
    resp = yield search.start_search(loop=False, timeout=50, spotings_req=3, speed=0.3)

    if resp is None or not resp.found:
        fprint("No gate found...", msg_color="red")
        defer.returnValue(None)

    position = rosmsg_to_numpy(resp.pose.pose.position)
    orientation = rosmsg_to_numpy(resp.pose.pose.orientation)

    distance = np.linalg.norm(sub.pose.position - position)
    gate_dst = GATE_DISTANCE - distance

    # normal = tf.transformations.quaternion_matrix(
    #             orientation).dot(np.array([0, 1, 0, 0]))[0:3]
    normal = np.linalg.norm(sub.pose.position - position)
    where_to_go_before_style = sub.pose.position + gate_dst * 0.5 * normal
    where_to_go_after_style = sub.pose.position + gate_dst * normal

    yield sub.move.set_position(where_to_go_before_style).look_at(where_to_go_after_style).zero_roll_and_pitch().go(speed=SPEED)

    fprint('YOLO! With style')
    yield style_points(sub)

    fprint('YOLO! Going through gate')
    yield sub.move.set_position(where_to_go_after_style).zero_roll_and_pitch().go(speed=SPEED)

    yield sub.vision_proxies.start_gate.stop()

    defer.returnValue(True)


@txros.util.cancellableInlineCallbacks
def style_points(sub):
    yield sub.move.yaw_right(np.pi / 2).go()
    yield sub.move.yaw_right(np.pi / 2).go()
    yield sub.move.yaw_right(np.pi / 2).go()
    yield sub.move.yaw_right(np.pi / 2).go()
    yield sub.move.zero_roll_and_pitch().go()
