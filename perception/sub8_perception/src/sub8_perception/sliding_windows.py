import cv2
import numpy as np
from feature_extraction import extract_features_single
from skimage.feature import hog

def draw_boxes(img, bboxes, color=(255, 165, 0), thick=3):
    # Make a copy of the image
    imcopy = np.copy(img)
    # Iterate through the bounding boxes
    for bbox in bboxes:
        # Draw a rectangle given bbox coordinates
        cv2.rectangle(imcopy, bbox[0], bbox[1], color, thick)
    # Return the image copy with boxes drawn
    return imcopy

def slide_window(img, x_start_stop=[None, None], y_start_stop=[None, None], 
                    xy_window=(64, 64), xy_overlap=(0.5, 0.5)):
    # If x and/or y start/stop positions not defined, set to image size
    if x_start_stop[0] == None:
        x_start_stop[0] = 0
    if x_start_stop[1] == None:
        x_start_stop[1] = img.shape[1]
    if y_start_stop[0] == None:
        y_start_stop[0] = 0
    if y_start_stop[1] == None:
        y_start_stop[1] = img.shape[0]
    # Compute the span of the region to be searched    
    xspan = x_start_stop[1] - x_start_stop[0]
    yspan = y_start_stop[1] - y_start_stop[0]
    # Compute the number of pixels per step in x/y
    nx_pix_per_step = max(1, np.int(xy_window[0]*(1 - xy_overlap[0])))
    ny_pix_per_step = max(1, np.int(xy_window[1]*(1 - xy_overlap[1])))
    # Compute the number of windows in x/y
    nx_buffer = np.int(xy_window[0]*(xy_overlap[0]))
    ny_buffer = np.int(xy_window[1]*(xy_overlap[1]))
    nx_windows = np.int((xspan-nx_buffer)/nx_pix_per_step) 
    ny_windows = np.int((yspan-ny_buffer)/ny_pix_per_step) 
    # Initialize a list to append window positions to
    window_list = []
    # Loop through finding x and y window positions
    for ys in xrange(ny_windows):
        for xs in xrange(nx_windows):
            # Calculate window position
            startx = xs*nx_pix_per_step + x_start_stop[0]
            endx = startx + xy_window[0]
            starty = ys*ny_pix_per_step + y_start_stop[0]
            endy = starty + xy_window[1]
            # Append window position to list
            if endy >= img.shape[0]:
                return window_list
            window_list.append(((startx, starty), (endx, endy)))
    # Return the list of windows
    return window_list

def mean_std_cr(img):
    ycrcb = cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
    cr = ycrcb[:,:,1]
    return np.mean(cr), np.std(cr)

def search_windows(img, windows, clf, scaler, color_space='YCrCb', spatial_size=(32,32),
                  hist_bins=32, orient=9, pix_per_cell=8, cell_per_block=2, hog_channel='ALL',
                  spatial_feat=False,hist_feat=True,hog_feat=True):
    #1) Create an empty list to receive positive detection windows
    on_windows = []
    # sat = sat_thresh(img)
    #2) Iterate over all windows in the list
    ycrcb = cv2.cvtColor(img, cv2.COLOR_RGB2YCrCb)
    cr = ycrcb[:,:,1]
    mean, std = mean_std_cr(img)
    for i in xrange(len(cr)):
        for j in xrange(len(cr[0])):
            if cr[i][j] > mean - 1.6 * std and cr[i][j] < mean + 1.6 * std:
                cr[i][j] = 0
    for window in windows:
        if cr[int((window[0][1] + window[1][1]) / 2)][int((window[0][0] + window[1][0]) / 2)] == 0:
            continue
        #3) Extract the test window from original image
        test_img = cv2.resize(img[window[0][1]:window[1][1], window[0][0]:window[1][0]], (64, 64))      
        #4) Extract features for that window using single_img_features()
        # features = get_hog_features(test_img)
        features = extract_features_single(test_img, color_space=color_space, spatial_size=spatial_size, hist_bins=hist_bins,
                         orient=orient, pix_per_cell=pix_per_cell, cell_per_block=cell_per_block,
                         hog_channel=hog_channel, spatial_feat=spatial_feat, hist_feat=hist_feat,
                         hog_feat=hog_feat)
        #5) Scale extracted features to be fed to classifier
        test_features = scaler.transform(np.array(features).reshape(1, -1))
        #6) Predict using your classifier
        prediction = clf.predict(test_features)
        #7) If positive (prediction == 1) then save the window
        if prediction == 1:
            on_windows.append(window)
    #8) Return windows for positive detections
    return on_windows

def add_heat(heatmap, bbox_list, add=True):
    # Iterate through list of bboxes
    if add:
        for box in bbox_list:
            # Add += 1 for all pixels inside each bbox
            # Assuming each "box" takes the form ((x1, y1), (x2, y2))
            heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] += 1
    else:
        for box in bbox_list:
            # Subtract -= 1 for all pixels inside each bbox
            # Assuming each "box" takes the form ((x1, y1), (x2, y2))
            heatmap[box[0][1]:box[1][1], box[0][0]:box[1][0]] -= 1  
    # Return updated heatmap
    return heatmap# Iterate through list of bboxes
    
def apply_threshold(heatmap, threshold):
    # Zero out pixels below the threshold
    heatmap[heatmap <= threshold] = 0
    # Return thresholded map
    return heatmap

def draw_labeled_bboxes(img, labels):
    # Iterate through all detected objects
    for obj_num in xrange(1, labels[1]+1):
        # Find pixels with each object_number label value
        nonzero = (labels[0] == obj_num).nonzero()
        # Identify x and y values of those pixels
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Define a bounding box based on min/max x and y
        bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
        # Draw the box on the image
        cv2.rectangle(img, bbox[0], bbox[1], (255,165,0), 6)
    # Return the image
    return img