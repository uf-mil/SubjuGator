#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from mil_ros_tools import Image_Publisher, Image_Subscriber

class CameraEqualization:
	def __init__(self):
		self.image_sub_l = Image_Subscriber('/camera/front/left/image_rect_color', self._img_callback_l)
		self.image_sub_r = Image_Subscriber('/camera/front/right/image_rect_color', self._img_callback_r)
		self.image_pub_equalize_l = Image_Publisher('/camera/front/left/image_rect_equalized')
		self.image_pub_equalize_r = Image_Publisher('/camera/front/right/image_rect_equalized')

	def _img_callback_l(self, img):
		image_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
		l, a, b = cv2.split(image_lab)
		clahe = cv2.createCLAHE(clipLimit=10, tileGridSize=(8,8))
		cl = clahe.apply(l)
		equ_img_lab = cv2.merge((cl,a,b))
		self.image_pub_equalize_l.publish(cv2.cvtColor(equ_img_lab, cv2.COLOR_LAB2BGR))

	def _img_callback_r(self, img):
		image_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
		l, a, b = cv2.split(image_lab)
		clahe = cv2.createCLAHE(clipLimit=10, tileGridSize=(8,8))
		cl = clahe.apply(l)
		equ_img_lab = cv2.merge((cl,a,b))
		self.image_pub_equalize_r.publish(cv2.cvtColor(equ_img_lab, cv2.COLOR_LAB2BGR))



if __name__ == '__main__':
    rospy.init_node('camera_equalization')
    CameraEqualization()
    rospy.spin()
