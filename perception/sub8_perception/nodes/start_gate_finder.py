#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from mil_ros_tools import numpy_to_point, Image_Publisher, Image_Subscriber, numpy_to_quaternion
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped, Point, Vector3Stamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class StartGateFinder():
	def __init__(self):
		self.image_sub = Image_Subscriber('/camera/front/right/image_rect_color', self._img_callback_a)
		self.image_pub = Image_Publisher('/start_gate_finder/bgr')
		self.image_pub_equalize = Image_Publisher('/camera/front/right/image_rect_equalized')
		self.image_pub_black = rospy.Publisher("/start_gate_finder/masks",Image)
		self.last_bgr = None
		self.vis = None
		self.bridge = CvBridge()


	def _get_angle(self, a, b, c):
		ba = a[0]-b[0]
		bc = c[0]-b[0]
		cos_ang = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
		angle = np.arccos(cos_ang)
		return np.degrees(angle)

	def _valid_contour(self, contour):
		area = cv2.contourArea(contour)
		if(area < 1000):
			# print "failed area 1000"
			return False
		# 310408
		if(area > 30000):
			print "failed area 30000"
			return False

		epsilon = 0.01*cv2.arcLength(contour,True)
		approx = cv2.approxPolyDP(contour,epsilon,True)
		rect = cv2.minAreaRect(approx)


		M = cv2.moments(contour)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		center= (cX, cY)
		if cv2.pointPolygonTest(contour, center, False) == 1:
			print "center is outside contour"
			return False

		# if(rect[1][0]*rect[1][1] < 7000):
		# 	print "failed rect size"
		# 	return False;


		if(len(approx) < 4):
			print "less than 8"
			return False
		if(len(approx) > 15):
			print "greater than 15"
		# 	return False;
		# cv2.drawContours(self.last_bgr, approx, -1, (0, 255, 255), 5)
		for i in range(0, len(approx)-2, 2):
			angle = self._get_angle(approx[i], approx[i+1], approx[i+2])
			if(np.abs(angle - 90) > 30):
				print "small angle"
				print angle
				return False
		angle = self._get_angle(approx[len(approx)-1], approx[0], approx[1])
		if(np.abs(angle - 90) > 30):
			print "small angle " 
			print angle
			return False

		return True


	def _get_edges(self, image):
		image = cv2.blur(image, (5, 5))
		# _, blur = cv2.threshold(blur,140,255,cv2.THRESH_TRUNC)
		# cv2.imshow("blur", blur)
		# cv2.imshow("img", self.last_lab)
		# cv2.imshow("test", cv2.Canny(self.last_lab, 20, 20 * 3.0))
		# cv2.imshow("blur", image)
		# 
		kernel = np.ones((5,5),np.uint8)
		canny = cv2.Canny(image, 35, 35*3)
		cv2.imshow("sijdis", canny)
		closing = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
		dilation = cv2.dilate(canny,kernel,iterations = 3)
		# cv2.imshow("dial", dilation)
		conc_black = np.concatenate((canny, closing, dilation), axis=1)
		self.image_pub_black.publish(self.bridge.cv2_to_imgmsg(conc_black, "mono8"))
		return dilation

	def _img_callback_a(self, img):
		self.last_bgr = img
		image = img
		# Pool/red thing:
		image_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
		image_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		image_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		image_ycr = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
		# image = cv2.equalizeHist(image_gray)
		image_ycr[:,:,0] = cv2.equalizeHist(image_ycr[:,:,0]);
		# (thresh, im_bw) = cv2.threshold(image_gray, 130, 150, cv2.THRESH_BINARY)
		# image_bgr_g = image[:,:,1] * 1
		# image_lab_a = image_lab[:,:,0]
		#image_hsv_s = image_hsv[:,:,0]
		# image = cv2.bitwise_or(image_lab_a, image_bgr_g)
		# image = cv2.bitwise_or(im_bw, img[:,:,1])
		# cv2.imshow("sidjsid", image_ycr[:,:,2])

		image = cv2.cvtColor(image_ycr,cv2.COLOR_YCrCb2BGR);
		self.image_pub_equalize.publish(image)


		#-----Converting image to LAB Color model----------------------------------- 

		#-----Splitting the LAB image to different channels-------------------------
		l, a, b = cv2.split(image_lab)

		#-----Applying CLAHE to L-channel-------------------------------------------
		clahe = cv2.createCLAHE(clipLimit=10, tileGridSize=(16,16))
		cl = clahe.apply(l)
		cv2.imshow('CLAHE output', cl)
		# (thresh, image) = cv2.threshold(cl, 97,150, cv2.THRESH_BINARY_INV)
		# cv2.imshow("threshold", image)
		image = cl
		#-----Merge the CLAHE enhanced L-channel with the a and b channel-----------
		# limg = cv2.merge((cl,a,b))
		#-----Converting image from LAB Color model to RGB model--------------------
		# image = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
		# cv2.imshow('final', image)

		# cv2.imshow("gray", im_bw)
		#Transdeck /yellow thing:
		# image = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

		edges = self._get_edges(image)
		_, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for idx, c in enumerate(contours):
			if(self._valid_contour(c)):
				cv2.drawContours(img, contours, idx, (200, 0, 0), 3)
				M = cv2.moments(c)
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				cv2.circle(img, (cX, cY), 3, (200, 0, 0), -1)
				print("Found")
				# cv2.drawContours(self.last_bgr,[box],0,(0,0,255),2)
		
		cv2.imshow("found", img)
		# self.vis = np.concatenate((image, img), axis=1)
		# self.image_pub.publish(self.vis)
		# cv2.imshow("debug", img)
		cv2.waitKey(30)

if __name__ == '__main__':
    rospy.init_node('start_gate_finder')
    StartGateFinder()
    rospy.spin()
