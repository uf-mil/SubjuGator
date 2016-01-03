#pragma once  // header guard
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include "ros/ros.h"

namespace slam {

typedef cv::Point2f Point2;
typedef cv::Point3f Point3;
typedef std::vector<Point2> PointVector;
typedef std::vector<Point3> Point3Vector;
// typedef std::vector<cv::Mat(4, 1, CV_32FC1)> Point4Vector;
typedef std::vector<cv::Mat> Point4Vector;
typedef std::vector<uchar> StatusVector;
typedef std::vector<int> IdVector;

// Undistort the image, downsample by half
void preprocess(cv::Mat& input_image, cv::Mat& output_image, const cv::Mat& intrinsics,
                const cv::Mat& distortion);

void initialize(const cv::Mat& frame, PointVector& corners, IdVector& feature_ids);

PointVector filter(const std::vector<uchar>& status, const PointVector& points);

void optical_flow(const cv::Mat& prev_frame, const cv::Mat& cur_frame, PointVector& prev_pts,
                  PointVector& next_pts, std::vector<uchar>& status);

void draw_points(cv::Mat& frame, const PointVector& points);

// ******* Motion *******
cv::Mat estimate_fundamental_matrix(const PointVector& pts_1, const PointVector& pts_2,
                                    std::vector<uchar>& inliers);
cv::Mat estimate_motion(const PointVector& pts_1, const PointVector& pts_2, const cv::Mat& F,
                        const cv::Mat& K);
void triangulate(const cv::Mat pose_1, const cv::Mat pose_2, const cv::Mat K,
                 const PointVector& pts_1, const PointVector& pts_2, Point3Vector& triangulated);

std::vector<int> which_points(const StatusVector& status, const std::vector<int>& previous);
void visualize_point3vector(const Point3Vector& point3vector);
void draw_point_ids(cv::Mat& frame, const PointVector& points, const std::vector<int>& point_ids);
class RvizVisualizer {
public:
  ros::Publisher camera_pub;
  ros::Publisher point_pub;
  ros::NodeHandle nh;
  RvizVisualizer();
  void create_marker(visualization_msgs::Marker& marker);
  void draw_points(Point3Vector points);
};
}