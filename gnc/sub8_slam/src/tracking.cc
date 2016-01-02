#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sub8_slam/slam.h>

// #define UNDISTORT

namespace slam {

void preprocess(cv::Mat& input_image, cv::Mat& output_image, const cv::Mat& intrinsics,
                const cv::Mat& distortion) {
  /* Resize and undistort input image
      -- Destroys the input image in the process
  */
  cv::cvtColor(input_image, input_image, CV_BGR2GRAY);
  cv::resize(input_image, input_image, cv::Size(input_image.cols / 2, input_image.rows / 2));
#ifdef UNDISTORT
  cv::undistort(input_image, output_image, intrinsics, distortion);
#else
  // Resize only
  output_image = input_image;
#endif
}

void initialize(const cv::Mat& frame, PointVector& corners) {
  // Parameters to twiddle
  // These should NOT be exposed
  int feature_mask_offset = 5;
  int max_corners = 2000;
  double quality = 0.25;
  double min_distance = 7;
  int block_size = 5;      // 5
  bool use_harris = true;  // Use harris corner detector?

  // Create feature mask (Only search here for features)
  cv::Mat feature_mask = cv::Mat(frame.rows, frame.cols, CV_8UC1, 1);
  // cv::Point2i lower_right(feature_mask_offset, feature_mask_offset);
  // cv::Point2i upper_right(frame.cols - feature_mask_offset, frame.rows - feature_mask_offset);
  // cv::rectangle(feature_mask, lower_right, upper_right, cv::Scalar(1));

  // Discover features
  cv::goodFeaturesToTrack(frame, corners, max_corners, quality, min_distance, feature_mask,
                          block_size, use_harris);
}

std::vector<int> which_points(const std::vector<uchar>& status, const std::vector<int>& previous) {
  /* Figure out which points from the original appear in the current frame, numbered */
  std::vector<int> point_associations;
  if (previous.size() == 0) {
    for (unsigned int k = 0; k < status.size(); k++) {
      if (status[k]) {
        point_associations.push_back(k);
      }
    }
  } else {
    for (unsigned int k = 0; k < status.size(); k++) {
      if (status[k]) {
        point_associations.push_back(previous[k]);
      }
    }
  }
}

PointVector filter(const std::vector<uchar>& status, const PointVector& points) {
  /* filtered_points should be empty

  */
  PointVector filtered_points;
  for (unsigned int k = 0; k < status.size(); k++) {
    if (status[k]) {
      filtered_points.push_back(points[k]);
    } else {
      std::cout << bool(status[k]) << std::endl;
    }
  }
  return filtered_points;  // All hail C++11 move semantics!
}

void optical_flow(const cv::Mat& prev_frame, const cv::Mat& cur_frame, PointVector& prev_pts,
                  PointVector& next_pts, std::vector<uchar>& status) {
  /* Compute best sparse optical flow, Lukas-Kanade */
  // Parameters (Not to be exposed!)
  cv::Size win_size(7, 7);  // (21, 21)
  int max_level = 2;
  cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 20, 0.03);

  // Declarations
  // PointVector next_pts;
  // std::vector<uchar> status;
  std::vector<float> error;
  cv::calcOpticalFlowPyrLK(prev_frame, cur_frame, prev_pts, next_pts, status, error, win_size,
                           max_level, criteria);
}

void draw_points(cv::Mat& frame, const PointVector& points) {
  int radius = 5;
  for (unsigned int k = 0; k < points.size(); k++) {
    cv::circle(frame, points[k], radius, cv::Scalar(240), 1);
    // todo: red rectangle outlines would look cooler
  }
}
}