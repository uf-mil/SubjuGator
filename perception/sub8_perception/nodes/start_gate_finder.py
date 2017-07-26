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
		img = image
		image = cv2.medianBlur(image, 5)
		# image = cv2.blur(image, (5, 5))
		# _, blur = cv2.threshold(blur,140,255,cv2.THRESH_TRUNC)
		cv2.imshow("blur", image)
		# cv2.imshow("img", self.last_lab)
		# cv2.imshow("test", cv2.Canny(self.last_lab, 20, 20 * 3.0))
		# cv2.imshow("blur", image)

		kernel = np.ones((5,5),np.uint8)
		canny = cv2.Canny(image, 60, 60*3)
		closing = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
		dilation = cv2.dilate(canny,kernel,iterations = 4)
		cv2.imshow("sijdis", dilation)
		# lines = cv2.HoughLinesP(canny, 1, np.pi/2, 2, minLineLength = 620, maxLineGap = 100)[0].tolist()
		lines_vertical = cv2.HoughLinesP(dilation, 1, np.pi, threshold=150, minLineLength=70, maxLineGap=80)
		verts = []
		try:
			lines = lines_vertical
			img_line_mask = np.zeros(img.shape, dtype=np.uint8)
			if lines is None:
				return
			for x in range(0, len(lines)):
				for x1,y1,x2,y2 in lines[x]:
					angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi
					if np.abs(angle) > 20 and np.abs(angle) < 70:
						continue
					print angle
			 		cv2.line(img_line_mask,(x1,y1),(x2,y2),(255),2)
			cv2.imshow("mask", img_line_mask)
		except:
			print "rip"
		canny_mask = cv2.Canny(img_line_mask, 10, 10*3)
		cv2.imshow("c_mask", canny_mask)
		dialte_mask = cv2.dilate(canny_mask,kernel,iterations = 2)
		_, contours_thresh, _ = cv2.findContours(dialte_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for idx, c in enumerate(contours_thresh):
			epsilon = 0.1*cv2.arcLength(c,True)
			approx = cv2.approxPolyDP(c,epsilon,True)
			cv2.drawContours(self.last_bgr, [approx], -1, (0, 0, 200), 3)

		# img_line_mask = cv2.dilate(img_line_mask, kernel, iterations = 6)
		# test = cv2.bitwise_and(image, img_line_mask)
		# (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(test)
		# cv2.circle(self.last_bgr, maxLoc, 10, (255, 0, 0), 2)
		# cv2.circle(test, maxLoc, 10, (255), 2)
		

		# thresh, heh = cv2.threshold(test, 110, 190, cv2.THRESH_BINARY)
		# canny_thresh = cv2.Canny(heh, 10, 10 * 3)
		# dilation_thresh = cv2.dilate(canny_thresh,kernel,iterations = 2)
		# _, contours_thresh, _ = cv2.findContours(dilation_thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


		# cv2.imshow("test", dilation_thresh)

		# if lines is None:
		# 	return
		# for x in range(0, len(lines)):
		# 	for x1,y1,x2,y2 in lines[x]:
		#    		cv2.line(self.last_bgr,(x1,y1),(x2,y2),(0,0,255),2)

		

		# cv2.imshow("dial", dilation)
		# conc_black = np.concatenate((canny, closing, dilation), axis=1)
		# self.image_pub_black.publish(self.bridge.cv2_to_imgmsg(test, "mono8"))

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
		limg = cv2.merge((cl,a,b))
		#-----Converting image from LAB Color model to RGB model--------------------
		image_clahe = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
		image_b = cv2.medianBlur(image_clahe,5)
		cv2.imshow("original", image_clahe)
		cv2.imshow("blur", image_b)
		# cv2.imshow('final', image)

		# cv2.imshow("gray", im_bw)
		#Transdeck /yellow thing:
		# image = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

		edges = self._get_edges(image)
		_, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for idx, c in enumerate(contours):
			if(self._valid_contour(c)):
				M = cv2.moments(c)
				cv2.drawContours(self.last_bgr, contours, idx, (200, 0, 0), 3)
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				cv2.circle(self.last_bgr, (cX, cY), 3, (200, 0, 0), -1)
				print("Found")
				# cv2.drawContours(self.last_bgr,[box],0,(0,0,255),2)
		
		cv2.imshow("found", self.last_bgr)
		# self.vis = np.concatenate((image, img), axis=1)
		# self.image_pub.publish(self.vis)
		# cv2.imshow("debug", img)
		cv2.waitKey(30)

if __name__ == '__main__':
    rospy.init_node('start_gate_finder')
    StartGateFinder()
    rospy.spin()
