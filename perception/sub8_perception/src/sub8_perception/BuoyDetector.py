import cv2
from collections import OrderedDict
import glob
from keras.models import load_model
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage.measurements import label
from sklearn.externals import joblib
import sliding_windows
import time

# TODO: put features into separate class, integrate with ROS
class BuoyDetector:
	def __init__(self, svc, scaler, neural_net, color_space='YCrCb', 
				 spatial_size=(32, 32), hist_bins=32, orient=9, pix_per_cell=8,
				 cell_per_block=2, hog_channel='ALL', spatial_feat=False, hist_feat=False,
				 hog_feat=True):
		self.svc = svc
		self.scaler = scaler
		self.neural_net = neural_net
		self.buoy_locations = {'yellow': None, 'red': None, 'green': None}
		self.color_codes = {'yellow': 0, 'red': 1, 'green': 2}
		self.color_space = color_space
		self.spatial_size = spatial_size
		self.hist_bins = hist_bins
		self.orient = orient
		self.pix_per_cell = pix_per_cell
		self.cell_per_block = cell_per_block
		self.hog_channel = hog_channel
		self.spatial_feat = spatial_feat
		self.hist_feat = hist_feat
		self.hog_feat = hog_feat

	def find_buoys(self, img):
		# blind search
		if self.buoy_locations['yellow'] is None and self.buoy_locations['red'] is None and self.buoy_locations['green'] is None:
			self.blind_search(img)
		else:
			for color in self.buoy_locations.keys():
				self.next_buoy_location(img=img, color=color)
		if self.buoy_locations['yellow'] is None or self.buoy_locations['red'] is None or self.buoy_locations['green'] is None:
			self.local_search(img=img)
		return self.buoy_locations

	# TODO: widen rectangle, subsample feature vector to improve runtime
	def blind_search(self, img):
		total_on_windows = []
		window_len = [32, 64, 96, 128]
		for l in window_len:
			window_list = sliding_windows.slide_window(img=img, xy_window=(l, l), xy_overlap=(0.9, 0.9))
			on_windows = sliding_windows.search_windows(img=img, windows=window_list, clf=self.svc, scaler=self.scaler,
														color_space=self.color_space, spatial_size=self.spatial_size, hist_bins=self.hist_bins, orient=self.orient,
														pix_per_cell=self.pix_per_cell, cell_per_block=self.cell_per_block, hog_channel=self.hog_channel, spatial_feat=self.spatial_feat,
														hist_feat=self.hist_feat, hog_feat=self.hog_feat)
			total_on_windows += on_windows
		heat = np.zeros_like(img[:,:,0]).astype(np.float)
		heat = sliding_windows.add_heat(heat, total_on_windows)
		thresh = 0
		heat = sliding_windows.apply_threshold(heat, thresh)
		heatmap = np.clip(heat, 0, 255)
		labels = label(heatmap)
		if labels[1] == 0:
			return self.buoy_locations
		img_segs, img_segs_locs = [], []
		for obj_num in range(1, labels[1] + 1):
			nonzero = (labels[0] == obj_num).nonzero()
			nonzeroy = np.array(nonzero[0])
			nonzerox = np.array(nonzero[1])
			min_x, max_x = np.min(nonzerox), np.max(nonzerox)
			min_y, max_y = np.min(nonzeroy), np.max(nonzeroy)
			l = min((max_x - min_x, max_y - min_y))
			disp = 0
			if l == max_x - min_x:
				while min_y + disp + l <= max_y:
					img_seg = img[(min_y + disp):(min_y + disp + l), min_x:max_x]
					disp += 2
					img_seg = cv2.resize(img_seg, (64, 64))
					img_segs.append(img_seg)
					img_segs_locs.append(((min_x, min_y + disp), (max_x, min_y + disp + l)))
			else:
				while min_x + disp + l <= max_x:
					img_seg = img[min_y:max_y, (min_x + disp):(min_x + disp + l)]
					disp += 2
					img_seg = cv2.resize(img_seg, (64, 64))
					img_segs.append(img_seg)
					img_segs_locs.append(((min_x + disp, min_y), (min_x + disp + l, max_y)))
		img_segs = np.asarray(img_segs)
		self.update_buoy_locations(img_segs, img_segs_locs)

	def local_search(self, img):
		for location in self.buoy_locations.itervalues():
			if location is not None:
				l = location[1][0] - location[0][0]
				height = 2 * l
				width = 6 * l
				centroid = (int((location[0][0] + location[1][0]) / 2), int((location[0][1] + location[1][1]) / 2))
				upper_left = (max(int(centroid[0] - width / 2), 0), max(int(centroid[1] - height / 2), 0))
				bottom_right = (min(int(centroid[0] + width / 2), img.shape[1] - 1), min(int(centroid[1] + height / 2), img.shape[0] - 1))
				windows = sliding_windows.slide_window(img=img, x_start_stop=[upper_left[0], bottom_right[0]], y_start_stop=[upper_left[1], bottom_right[1]],
														   xy_window=(l, l), xy_overlap=(0.9, 0.9))
				img_segs, img_segs_locs = self.image_segments_and_locations(img=img, windows=windows)
				self.update_buoy_locations(img_segs, img_segs_locs)
				return

	def update_buoy_locations(self, img_segs, img_segs_locs):
		assert len(img_segs) == len(img_segs_locs)
		if len(img_segs) == 0:
			return
		softmaxes = self.neural_net.predict(img_segs)
		buoy_softmax = {'yellow': 0, 'red': 0, 'green': 0}
		for i, softmax in enumerate(softmaxes):
			pred = np.argmax(softmax)
			if softmax[pred] < 0.95:
				continue
			if pred == 0 and (softmax[pred] > buoy_softmax['yellow']):
				buoy_softmax['yellow'] = softmax[pred]
				self.buoy_locations['yellow'] = img_segs_locs[i]
			elif pred == 1 and (softmax[pred] > buoy_softmax['red']):
				buoy_softmax['red'] = softmax[pred]
				self.buoy_locations['red'] = img_segs_locs[i]
			elif pred == 2 and (softmax[pred] > buoy_softmax['green']):
				buoy_softmax['green'] = softmax[pred]
				self.buoy_locations['green'] = img_segs_locs[i]
		# print(buoy_softmax)

	def next_buoy_location(self, img, color):
		color = color.lower()
		assert color == 'yellow' or color == 'red' or color == 'green'
		if self.buoy_locations[color] is None:
			return
		color_code = self.color_codes[color]
		loc = self.buoy_locations[color]
		side_len = loc[1][0] - loc[0][0]
		disp = int(side_len / 2)
		x_start = max(loc[0][0] - disp, 0)
		y_start = max(loc[0][1] - disp, 0)
		x_end = min(loc[1][0] + disp, img.shape[1] - 1)
		y_end = min(loc[1][1] + disp, img.shape[0] - 1)
		windows = sliding_windows.slide_window(img=img, x_start_stop=[x_start, x_end], y_start_stop=[y_start, y_end], 
											   xy_window=(side_len, side_len), xy_overlap=(0.9, 0.9))
		# print("Number of windows: {}".format(len(windows)))
		img_segs, img_segs_locs = self.image_segments_and_locations(img=img, windows=windows)
		softmaxes = self.neural_net.predict(img_segs)
		best_softmax, best_loc = 0, None
		for i, softmax in enumerate(softmaxes):
			if softmax[color_code] < 0.95:
				continue
			if softmax[color_code] > best_softmax:
				best_softmax = softmax[color_code]
				best_loc = img_segs_locs[i]
		self.buoy_locations[color] = best_loc

	def image_segments_and_locations(self, img, windows):
		img_segs, img_segs_locs = [], []
		for window in windows:
			img_seg = img[window[0][1]:window[1][1], window[0][0]:window[1][0]]
			img_seg = cv2.resize(img_seg, (64, 64))
			img_segs.append(img_seg)
			img_segs_locs.append(window)
		return np.asarray(img_segs), img_segs_locs

def import_images(directory):
	images = OrderedDict()
	file_extensions = ['.jpg', '.png']
	for file_extension in file_extensions:
	    for filename in sorted(glob.glob(directory + '/*' + file_extension)):
	        img = Image.open(filename)
	        dirs = filename.split('/')
	        images[dirs[-1]] = np.array(img)
	return images

def show_boxed_buoys(img, bd):
	start = time.time()
	res = bd.find_buoys(img)
	end = time.time()
	print(end - start)
	for k, v in res.iteritems():
		if v is None:
			continue
		if k == 'yellow':
			img = sliding_windows.draw_boxes(img, [v], color=(255, 255, 0))
		elif k == 'green':
			img = sliding_windows.draw_boxes(img, [v], color=(0, 255, 0))
		elif k == 'red':
			img = sliding_windows.draw_boxes(img, [v], color=(255, 0, 0))
	plt.figure()
	plt.imshow(img)
	plt.show()

if __name__ == "__main__":
	test_images = import_images(directory='buoy_frames')
	svc = joblib.load('svc5.pkl')
	scaler = joblib.load('scaler5.pkl')
	neural_net = load_model('nn.h5')
	bd = BuoyDetector(svc=svc, scaler=scaler, neural_net=neural_net)
	for img in test_images.itervalues():
		show_boxed_buoys(img, bd)