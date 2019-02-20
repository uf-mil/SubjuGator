from txros import util
import numpy as np
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
import mil_ros_tools
from twisted.internet import defer
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker
from mil_misc_tools import text_effects
from std_srvs.srv import SetBool, SetBoolRequest
from sub8_msgs.srv import GuessRequest, GuessRequestRequest

fprint = text_effects.FprintFactory(title="DICE", msg_color="cyan").fprint

pub_cam_ray = None

SPEED = 0.1
SPEED_GO = 0.2


@util.cancellableInlineCallbacks
def search(sub):
    step = 0
    while True:
        if step == 0:
            yield sub.move.yaw_left_deg(30).go(speed=SPEED)
        if step == 1:
            yield sub.move.pitch_down_deg(15).go(speed=SPEED)
        if step == 2:
            yield sub.move.yaw_right_deg(60).go(speed=SPEED)
        if step == 3:
            yield sub.move.zero_roll_and_pitch().depth(2.2).go(speed=SPEED)
        if step == 4:
            yield sub.move.yaw_left_deg(30).go(speed=SPEED)
        step = (step + 1) % 5
        yield sub.nh.sleep(3)


@util.cancellableInlineCallbacks
def run(sub):
    global SPEED
    global pub_cam_ray
    fprint('Enabling cam_ray publisher')
    pub_cam_ray = yield sub.nh.advertise('/dice/cam_ray', Marker)

    dice_txros = yield sub.nh.get_service_client('/guess_location',
                                                 GuessRequest)
    dice_1_req = yield dice_txros(GuessRequestRequest(item='dice'))
    dice = mil_ros_tools.rosmsg_to_numpy(dice_1_req.location.pose.position)

    yield sub.nh.sleep(1)

    fprint('Going to dice')

    yield sub.move.depth(2).look_at_without_pitching(dice).go(speed=SPEED)
    yield sub.move.set_position(dice).depth(2).zero_roll_and_pitch().go(
        speed=SPEED_GO)
    yield sub.move.depth(2.2).go(speed=SPEED_GO)

    fprint('Connecting camera')

    cam_info_sub = yield sub.nh.subscribe('/camera/front/left/camera_info',
                                          CameraInfo)

    fprint('Obtaining cam info message')
    cam_info = yield cam_info_sub.get_next_message()
    model = PinholeCameraModel()
    model.fromCameraInfo(cam_info)

    enable_service = sub.nh.get_service_client("/vision/dice_detection/enable",
                                               SetBool)
    yield enable_service(SetBoolRequest(data=True))

    dice_sub = yield sub.nh.subscribe('/dice/points', Point)

    found = {}
    history_tf = {}
    s = search(sub)
    while len(found) != 2:
        fprint('Getting dice xy')
        dice_xy = yield dice_sub.get_next_message()
        found[dice_xy.z] = mil_ros_tools.rosmsg_to_numpy(dice_xy)[:2]
        fprint(found)
        out = yield get_transform(sub, model, found[dice_xy.z])
        history_tf[dice_xy.z] = out
        if len(found) > 1:
            tmp = found.values()[0] - found.values()[1]
            # Make sure the two points are at least 100 pixels off
            diff = np.hypot(tmp[0], tmp[1])
            fprint('Distance between buoys {}'.format(diff))
            if diff < 40:
                found = {}
    # too lazy to deal with tx
    try:
        s.cancel()
    except Exception:
        fprint('oof', msg_color='yellow')

    yield enable_service(SetBoolRequest(data=False))

    start = sub.move.zero_roll_and_pitch()
    yield start.go()

    for i in range(2):
        fprint('Hitting dice {}'.format(i))
        # Get one of the dice
        dice, xy = found.popitem()
        fprint('dice {}'.format(dice))
        ray, base = history_tf[dice]

        where = base + 3 * ray

        fprint(where)
        fprint('Moving!', msg_color='yellow')
        fprint('Current position: {}'.format(sub.pose.position))
        fprint('zrp')
        yield sub.move.zero_roll_and_pitch().go(blind=True)
        yield sub.nh.sleep(4)
        fprint('hitting', msg_color='yellow')
        yield sub.move.look_at(where).go(blind=True, speed=SPEED)
        yield sub.nh.sleep(4)
        yield sub.move.set_position(where).go(blind=True, speed=SPEED)
        yield sub.nh.sleep(4)
        fprint('going back', msg_color='yellow')
        yield start.go(blind=True, speed=SPEED)
        yield sub.nh.sleep(4)


@util.cancellableInlineCallbacks
def get_transform(sub, model, point):
    fprint('Projecting to 3d ray')
    ray = np.array(model.projectPixelTo3dRay(point))
    fprint('Transform')
    transform = yield sub._tf_listener.get_transform('/map', 'front_left_cam')
    ray = transform._q_mat.dot(ray)
    ray = ray / np.linalg.norm(ray)
    marker = Marker(
        ns='dice',
        action=visualization_msgs.Marker.ADD,
        type=Marker.ARROW,
        scale=Vector3(0.2, 0.2, 2),
        points=np.array([
            Point(transform._p[0], transform._p[1], transform._p[2]),
            Point(transform._p[0] + ray[0], transform._p[1] + ray[1],
                  transform._p[2] + ray[2]),
        ]))
    marker.header.frame_id = '/map'
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1
    global pub_cam_ray
    pub_cam_ray.publish(marker)
    fprint('ray: {}, origin: {}'.format(ray, transform._p))
    defer.returnValue((ray, transform._p))
