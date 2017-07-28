#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import tf
from tf.transformations import quaternion_from_euler
from std_srvs.srv import SetBool, SetBoolResponse
from sub8_msgs.srv import VisionRequestResponse, VisionRequest, VisionRequest2D, VisionRequest2DResponse
from geometry_msgs.msg import PointStamped, Point, Vector3Stamped
from mil_ros_tools import numpy_to_point, Image_Publisher, Image_Subscriber, numpy_to_quaternion
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker
from mil_msgs.srv import SetGeometry
from mil_vision_tools import RectFinder, ImageMux


__author__ = "Kevin Allen"


# Ensure opencv3 is used, needed for KalmanFilter
assert cv2.__version__[0] == '3'


class ColoredRectangleFinder():

    """
    Node which finds colored rectangular objects in image frame.
    This can be used for the path marker challenge and to detect
    the lid of the bins challenge. The node estimates the 2d and 3d
    position/orientation of this object and returns this estimate when service is called.

    Unit tests for this node is in test_path_marker.py

    Finding the marker works as follows:
    * blur image
    * threshold image mostly for highly saturated, colored/yellow/red objects
    * run canny edge detection on thresholded image
    * find contours on edge frame
    * filter contours to find those that may be contours by:
      * checking # of sides in appox polygon
      * checking ratio of length/width close to known model
    * estimates 3D pose using cv2.solvePnP with known object demensions and camera model
    * Use translation vector from PnP and direction vector from 2d contour for pose
    * Transform this frames pose into /map frame
    * Plug this frames pose in /map into a kalman filter to reduce noise

    TODO: Allow for two objects to be identifed at once, both filtered through its own KF
    """
    # Coordinate axes for debugging image
    REFERENCE_POINTS = np.array([[0, 0, 0],
                                 [0.3, 0, 0],
                                 [0, 0.3, 0],
                                 [0, 0, 0.3]], dtype=np.float)

    def __init__(self):
        self.debug_gui = True
        self.enabled = True
        self.cam = None
        # Decides whether to search internal contours or ignore.
        self.internal_active = False

        self.bin_count = 0
        self.required_bin_count = 2

        # Colors for checking valid external contours
        self.blue = (255, 0, 0)
        self.green = (0, 255, 0)
        self.red = (0, 0, 255)
        self.cyan = (255, 255, 0)
        self.magenta = (255, 0, 255)

        # Color for checking valid internal contours
        self.yellow = (0, 255, 255)
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)

        # Constants from launch config file
        self.find_internal = rospy.get_param("~find_internal", True)
        self.debug_ros = rospy.get_param("~debug_ros", True)
        self.color_space = rospy.get_param(
            "~color_space", "LAB")  # LAB, BGR, HSV
        self.thresh_low = rospy.get_param("~thresh_low", [175, 80, 100])
        self.thresh_high = rospy.get_param("~thresh_high", [255, 150, 150])
        self.thresh_low = np.array(self.thresh_low)
        self.thresh_high = np.array(self.thresh_high)
        self.min_contour_area = rospy.get_param("~min_contour_area", 5000)

        self.epsilon_range = rospy.get_param("~epsilon_range", [0.02, 0.1])
        self.epsilon_step = rospy.get_param("~epsilon_step", 0.02)
        self.shape_match_thresh = rospy.get_param("~shape_match_thresh", 0.9)
        self.min_found_count = rospy.get_param("~min_found_count", 10)
        self.timeout_seconds = rospy.get_param("~timeout_seconds", 2.0)
        # Default to scale model of path marker. Please use set_geometry service
        # to set to correct model of object.
        length = rospy.get_param("~length", 0.9)
        width = rospy.get_param("~width", 0.6)
        self.rect_model = RectFinder(length, width)
        self.do_3D = rospy.get_param("~do_3D", True)
        camera = rospy.get_param(
            "~image_topic",
            "/camera/down/left/image_rect_color")

        self.tf_listener = tf.TransformListener()

        # Create kalman filter to track 3d position and direction vector for
        # marker in /map frame
        self.state_size = 5  # X, Y, Z, DY, DX
        self.filter = cv2.KalmanFilter(self.state_size, self.state_size)
        self.filter.transitionMatrix = 1. * \
            np.eye(self.state_size, dtype=np.float32)
        self.filter.measurementMatrix = 1. * \
            np.eye(self.state_size, dtype=np.float32)
        self.filter.processNoiseCov = 1e-6 * \
            np.eye(self.state_size, dtype=np.float32)
        self.filter.measurementNoiseCov = 1e-4 * \
            np.eye(self.state_size, dtype=np.float32)
        self.filter.errorCovPost = 1. * \
            np.eye(self.state_size, dtype=np.float32)

        self.reset()
        self.service_set_geometry = rospy.Service(
            '~set_geometry', SetGeometry, self._set_geometry_cb)
        if self.debug_ros:
            self.debug_pub = Image_Publisher("~debug_image")
            self.markerPub = rospy.Publisher('~marker', Marker, queue_size=10)
        self.service2D = rospy.Service(
            '~2D', VisionRequest2D, self._vision_cb_2D)
        if self.do_3D:
            self.service3D = rospy.Service(
                '~pose', VisionRequest, self._vision_cb_3D)
        self.toggle = rospy.Service('~enable', SetBool, self._enable_cb)

        self.image_sub = Image_Subscriber(camera)
        self.camera_info = self.image_sub.wait_for_camera_info()
        assert self.camera_info is not None
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(self.camera_info)

        # Forrest Magic
        last_img = None
        while not rospy.is_shutdown():
            now_img = self.image_sub.last_image
            if now_img is last_img:
                continue
            self._img_cb(now_img)
            last_img = now_img

    def _set_geometry_cb(self, req):
        self.rect_model = RectFinder.from_polygon(req.model)
        self.reset()
        rospy.loginfo(
            "Resetting rectangle model to LENGTH=%f, WIDTH=%f",
            self.rect_model.length,
            self.rect_model.width)
        return {'success': True}

    def _send_debug_marker(self):
        '''
        Sends a rviz marker in the camera frame with the estimated pose of the object.
        This marker is a scaled cube with the demensions of the model.
        Only called if debug_ros param == True
        '''
        if self.last3d is None or not self.found:
            return
        m = Marker()
        m.header.frame_id = '/map'
        m.header.stamp = self.last_found_time_3D
        m.ns = "colored_rectangle"
        m.id = 0
        m.type = 1
        m.action = 0
        # Real demensions of path marker
        m.scale.x = self.rect_model.length
        m.scale.y = self.rect_model.width
        m.scale.z = 0.05
        m.pose.position = numpy_to_point(self.last3d[0])
        m.pose.orientation = numpy_to_quaternion(self.last3d[1])
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.r = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(self.timeout_seconds)
        self.markerPub.publish(m)

    def _enable_cb(self, x):
        if x.data != self.enabled:
            self.reset()
            self.tf_listener.clear()
        self.enabled = x.data
        return SetBoolResponse(success=True)

    def _vision_cb_3D(self, req):
        res = VisionRequestResponse()
        if self.last_found_time_3D is None or self.image_sub.last_image_time is None:
            res.found = False
            return res
        dt = (self.image_sub.last_image_time -
              self.last_found_time_3D).to_sec()
        if dt < 0 or dt > self.timeout_seconds:
            res.found = False
        elif (self.last3d is None or not self.enabled):
            res.found = False
        else:
            res.pose.header.frame_id = "/map"
            res.pose.header.stamp = self.last_found_time_3D
            res.pose.pose.position = numpy_to_point(self.last3d[0])
            res.pose.pose.orientation = numpy_to_quaternion(self.last3d[1])
            res.found = True
        return res

    def _vision_cb_2D(self, req):
        res = VisionRequest2DResponse()
        if (self.last2d is None or not self.enabled):
            res.found = False
        else:
            res.header.frame_id = self.cam.tfFrame()
            res.header.stamp = self.last_found_time_2D
            res.pose.x = self.last2d[0][0]
            res.pose.y = self.last2d[0][1]
            res.camera_info = self.camera_info
            res.max_x = self.camera_info.width
            res.max_y = self.camera_info.height
            if self.last2d[1][0] < 0:
                self.last2d[1][0] = -self.last2d[1][0]
                self.last2d[1][1] = -self.last2d[1][1]
            angle = np.arctan2(self.last2d[1][1], self.last2d[1][0])
            res.pose.theta = angle
            res.found = True
        return res

    def reset(self):
        self.last_found_time_2D = None
        self.last_found_time_3D = None
        self.last2d = None
        self.last3d = None
        self._clear_filter(None)

    def _clear_filter(self, state):
        '''
        Reset filter and found state. This will ensure that the object
        is seen consistently before vision request returns true
        '''
        rospy.loginfo("MARKER LOST")
        self.found_count = 0
        self.found = False
        self.last3d = None
        self.filter.errorCovPre = 1. * \
            np.eye(self.state_size, dtype=np.float32)
        if state is not None:
            self.found_count = 1
            state = np.array(state, dtype=np.float32)
            self.filter.statePost = state

    def _update_kf(self, xxx_todo_changeme):
        '''
        Updates the kalman filter using the pose estimation
        from the most recent frame. Also tracks time since last seen and how
        often is has been seen to set the boolean "found" for the vision request
        '''
        (x, y, z, dy, dx) = xxx_todo_changeme
        if self.last_found_time_3D is None:  # First time found, set initial KF pose to this frame
            self._clear_filter((x, y, z, dy, dx))
            self.last_found_time_3D = self.image_sub.last_image_time
            return
        dt = (self.image_sub.last_image_time -
              self.last_found_time_3D).to_sec()
        self.last_found_time_3D = self.image_sub.last_image_time
        if dt < 0 or dt > self.timeout_seconds:
            rospy.logwarn(
                "Timed out since last saw marker, resetting. DT={}".format(dt))
            self._clear_filter((x, y, z, dy, dx))
            return

        self.found_count += 1
        measurement = 1. * np.array([x, y, z, dy, dx], dtype=np.float32)
        self.filter.predict()
        estimated = self.filter.correct(measurement)
        if self.found_count > self.min_found_count:
            angle = np.arctan2(estimated[3], estimated[4])
            self.last3d = ((estimated[0], estimated[1], estimated[2]),
                           quaternion_from_euler(0.0, 0.0, angle))
            if not self.found:
                rospy.loginfo("Marker Found")
            self.found = True

    def shift(
            self,
            point,
            starting_frame='base_link',
            ending_frame='ball_dropper'):
        '''
        Return the transformation of a point between frames
        Useful for sub moves when it is desired to position a particular frame somewhere instead of
        base_link of the sub in /map
        '''
        tf_base_to_dropper = tf.TransformListener()

        # find tf between baselink and ball dropper
        translation, quaternion = tf_base_to_dropper.lookupTransform(
            starting_frame, ending_frame, rospy.get_rostime())

        # add that difference to the position in base link
        point.point.x = point.point.x - translation[0]
        point.point.y = point.point.y - translation[1]
        point.point.z = point.point.z - translation[2]

        return point

    def _get_pose_3D(self, corners):
        tvec, rvec = self.rect_model.get_pose_3D(
            corners, cam=self.cam, rectified=True)
        if tvec[2][0] < 0.3:  # Sanity check on position estimate
            rospy.logwarn("Marker too close, must be wrong...")
            return False
        rmat, _ = cv2.Rodrigues(rvec)
        vec = rmat[:, 0]

        # Convert position estimate and 2d direction vector to messages to they
        # can be transformed
        ps = PointStamped()
        ps.header.frame_id = self.cam.tfFrame()
        ps.header.stamp = self.image_sub.last_image_time
        ps.point = Point(*tvec)
        vec3 = Vector3Stamped()
        vec3.vector.x = vec[0]
        vec3.vector.y = vec[1]
        vec3.header.frame_id = self.cam.tfFrame()
        vec3.header.stamp = self.image_sub.last_image_time
        map_vec3 = None
        map_ps = None

        # Transform pose estimate to map frame
        try:
            tf.TransformListener()
            self.tf_listener.waitForTransform(
                '/map', ps.header.frame_id, ps.header.stamp, rospy.Duration(0.1))
            map_ps = self.tf_listener.transformPoint('/map', ps)

            try:
                map_ps = self.shift(map_ps)
            except tf.Exception as err:
                rospy.logwarn(
                    "Couldn't Transform /base_link to ball_dropper: {}".format(err))

            map_vec3 = self.tf_listener.transformVector3('/map', vec3)

        except tf.Exception as err:
            rospy.logwarn(
                "Could not transform {} to /map error={}".format(self.cam.tfFrame(), err))
            return False
        # Try to ensure vector always points the same way, so kf is not thrown
        # off at some angles
        if map_vec3.vector.y < 0.0:
            map_vec3.vector.y = -map_vec3.vector.y
            map_vec3.vector.x = -map_vec3.vector.x
        measurement = (
            map_ps.point.x,
            map_ps.point.y,
            map_ps.point.z,
            map_vec3.vector.y,
            map_vec3.vector.x)

        # Update filter and found state with the pose estimate from this frame
        self._update_kf(measurement)

        if self.debug_ros:
            # Draw coordinate axis onto object using pose estimate to project
            refs, _ = cv2.projectPoints(
                self.REFERENCE_POINTS, rvec, tvec, self.cam.intrinsicMatrix(), np.zeros(
                    (5, 1)))
            refs = np.array(refs, dtype=np.int)
            cv2.line(
                self.last_image,
                (refs[0][0][0],
                 refs[0][0][1]),
                (refs[1][0][0],
                 refs[1][0][1]),
                (0,
                 0,
                 255))  # X axis refs
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]),
                     (refs[2][0][0], refs[2][0][1]), (0, 255, 0))  # Y axis ref
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]),
                     (refs[3][0][0], refs[3][0][1]), (255, 0, 0))  # Z axis ref
            self._send_debug_marker()
        return True

    def _is_valid_contour(self, contour):
        '''
        Does various tests to filter out contours that are clearly not
        a valid colored rectangle.
        * run approx polygon, check that sides == 4
        * find ratio of length to width, check close to known ratio IRL
        '''
        if cv2.contourArea(contour) < self.min_contour_area:
            return False, self.red
        mismatch = self.rect_model.verify_contour(contour)
        if mismatch > self.shape_match_thresh:
            return False, self.blue
        # Checks that contour is 4 sided
        corners = self.rect_model.get_corners(
            contour,
            debug_image=self.last_image,
            epsilon_range=self.epsilon_range,
            epsilon_step=self.epsilon_step)
        if corners is None:
            return False, self.magenta
        self.last2d = self.rect_model.get_pose_2D(corners)
        self.last_found_time_2D = self.image_sub.last_image_time
        if self.do_3D:
            if self.internal_active:
                if not self._get_pose_3D(corners):
                    return False, self.cyan

        return True, self.green

    def _get_edges(self):
        '''
        Proccesses latest image to find edges by:
        blurring and thresholding for highly saturated orangish objects
        then runs canny on threshold images and returns canny's edges
        '''
        blur = cv2.blur(self.last_image, (5, 5))
        if self.color_space == 'BGR':
            colored = blur
        if self.color_space == 'HSV':
            colored = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        if self.color_space == 'LAB':
            colored = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)

        kernel = np.ones((3, 3), np.uint8)
        erosion = cv2.erode(colored, kernel, iterations=6)

        self.thresh_no_erosion = cv2.inRange(
            colored, self.thresh_low, self.thresh_high)
        thresh = cv2.inRange(erosion, self.thresh_low, self.thresh_high)

        return thresh

    def minimum_norm(self, center, features, means, channel=0):
        '''
        Among the provided features find the proximity to a predefined center

        Center of feature may be geometric (ie. the 2D coordinates of the center of a contour),
        or it may be proximity of a color to another

        @param center   : base value to compare set of means against
        @param features : the set of features who
        @param means    : the corresponding set means of given set of features
        @param channel  : channel to check along feature
        '''
        distances = []
        for mean in means:
            distances.append(np.linalg.norm(center[channel:] - mean[channel:]))
        min_idx = distances.index(min(distances))
        return features[min_idx]

    def center_contour(self, contour):
        ''' Find the geometric center of a provided contour. '''
        center = cv2.moments(contour)
        cX = int(center["m10"] / center["m00"])
        cY = int(center["m01"] / center["m00"])
        return np.array([cX, cY])

    def image_mux_today(self, images):
        ''' Plotting tool for debugging'''
        labels = ['Original', 'Threshed', 'Full Mask', None]
        # size = (960, 1280)
        size = (480, 640)
        t = ImageMux(
            size=size, border_color=(
                0, 0, 255), border_thickness=1, shape=(
                2, 2), labels=labels, text_scale=1, text_color=(
                255, 255, 0))
        for i in xrange(len(images)):
            t[i] = np.array(images[i])
        cv2.imshow('Kapu', t.image)

    def find_contour_parameters(self, contours):
        ''' return areas and geomtric centers of a given set of contours'''
        areas = []
        contour_centers = []

        for c in contours:
            areas.append(cv2.contourArea(c))
            contour_centers.append([self.center_contour(c)])

        return areas, contour_centers

    def _img_cb(self, img):
        potential_contours = []
        self.potential_masks = []
        if not self.enabled or self.cam is None:
            return
        self.last_image = img
        edges = self._get_edges()
        _, contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check if each contour is valid
        for idx, c in enumerate(contours):
            valid, color = self._is_valid_contour(c)
            if valid:
                potential_contours.append(c)
                self.potential_masks.append(
                    self._get_mask_from_contour(
                        contours, idx))
                if self.debug_ros:
                    cv2.drawContours(self.last_image, contours, idx, color, 3)
            else:
                if self.debug_ros:
                    cv2.drawContours(self.last_image, contours, idx, color, 3)

        images = [self.last_image, edges]

        # Find Internal Contours
        if self.find_internal:
            try:
                channel = 2
                self.bgr = img.copy()
                areas, contour_centers = self.find_contour_parameters(
                    potential_contours)
                self.max_internal_area = min(areas) - 10
                closest_color = np.array([0, 0, 180])  # BGR
                _, internal_contours, hierarchy = cv2.findContours(
                    self.thresh_no_erosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                masks, mean_color, full_mask = self._get_internal_masks_and_color(
                    internal_contours, hierarchy[0])
                # check internal masks! for  stuff
                best_mask = self.minimum_norm(
                    closest_color, masks, mean_color, channel)
                best_center_mask = self.center_contour(best_mask)
                contour_with_lid = self.minimum_norm(
                    best_center_mask, potential_contours, contour_centers)

                self.internal_active = True
                self._is_valid_contour(contour_with_lid)
                self.internal_active = False

                # best internal mask
                x, y, w, h = cv2.boundingRect(best_mask)
                cv2.rectangle(self.last_image, (x, y),
                              (x + w, y + h), self.black, 2)

                # best external mask
                x1, y1, w1, h1 = cv2.boundingRect(contour_with_lid)
                cv2.rectangle(self.last_image, (x1, y1),
                              (x1 + w1, y1 + h1), self.green, 2)
                images = [self.last_image, edges, full_mask]

            except ValueError as e:
                rospy.logerr('No Valid Contours: {}'.format(e))

        if self.debug_ros:
            self.debug_pub.publish(self.last_image)
        if self.debug_gui:
            self.image_mux_today(images)
            cv2.waitKey(5)

    def _get_mask_from_contour(self, contours, idx):
        mask = np.zeros(self.last_image.shape[:2], np.uint8)
        cv2.drawContours(mask, contours, idx, self.white, cv2.FILLED)

        return mask

    def _confirm_mask_within_larger_masks(self, inner_mask, outer_masks):
        ''' Verify that the potential inner mask is inside any of the outer masks

            @param inner_mask  : mask that could potentially be internal to an outer mask
            @param outer_masks : set of masks that the inner mask could potentially be inside of

        '''
        for outer_mask in outer_masks:

            plus = cv2.bitwise_or(outer_mask, inner_mask)
            if np.array_equal(outer_mask, plus):
                return True
        return False

    def _get_internal_masks_and_color(self, contours, hierarchy):
        '''
        Among the white bins, find the masks for the contours
        and return their corresponding saturation values.
        '''

        masks = []
        full_mask = np.zeros(self.last_image.shape[:2], np.uint8)
        mean_color = []
        for idx, component in enumerate(zip(contours, hierarchy)):
            currentContour = component[0]
            x, y, w, h = cv2.boundingRect(currentContour)
            currentMask = self._get_mask_from_contour(contours, idx)
            status = self._confirm_mask_within_larger_masks(
                currentMask, self.potential_masks)
            area = bool(50 <= cv2.contourArea(
                currentContour) < self.max_internal_area)
            if area and status:
                cv2.drawContours(
                    self.last_image, contours, idx, self.yellow, 3)
                masks.append(currentMask)
                mean_color.append(cv2.mean(self.bgr, mask=masks[-1])[:3])
                full_mask = full_mask + masks[-1]
        return masks, mean_color, full_mask


if __name__ == '__main__':
    rospy.init_node('colored_rectangle_finder')
    ColoredRectangleFinder()
    rospy.spin()
