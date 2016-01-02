#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sub8_slam/slam.h>

namespace slam {

cv::Mat estimate_fundamental_matrix(const PointVector& pts_1, const PointVector& pts_2,
                                    std::vector<uchar>& inliers) {
  // PointVector inlier_mask;
  cv::Mat F;
  F = cv::findFundamentalMat(pts_1, pts_2, cv::FM_RANSAC, 2, 0.999, inliers);
  return cv::Mat_<float>(F);
}

int num_in_front(const cv::Mat pose_1, const cv::Mat pose_2, const cv::Mat K,
                 const PointVector& pts_1, const PointVector& pts_2) {
  /*
    Compute the number of points in front, without dehomogenizing
  */
  // Point4Vector output_pts;
  cv::Mat proj_1 = cv::Mat_<float>(K * pose_1);
  cv::Mat proj_2 = cv::Mat_<float>(K * pose_2);
  cv::Mat output_pts = cv::Mat(4, pts_2.size(), CV_32F);
  cv::triangulatePoints(proj_1, proj_2, pts_1, pts_2, output_pts);

  int front_count = 0;
  // Count our friends in front of the camera
  // TODO: Check if this should be -Z b/c of triangulation frame
  front_count = (int)cv::sum((output_pts.row(2) > 0.0) / 255)[0];
  return front_count;
}

void convert_from_homogeneous4d(const cv::Mat& src, Point3Vector& output) {
  // Opencv convert sucked
  for (unsigned int k; k < src.cols; k++) {
    cv::Mat row = src.col(k);
    output.push_back(Point3(row.at<float>(0) / row.at<float>(3),
                            row.at<float>(1) / row.at<float>(3),
                            row.at<float>(2) / row.at<float>(3)));
  }
}

cv::Mat deskew(const cv::Mat T) {
  // Double check row or column major
  cv::Mat t = (cv::Mat_<float>(3, 1) << T.at<float>(2, 1), T.at<float>(0, 2), T.at<float>(1, 0));
  return t;
}

void triangulate(const cv::Mat pose_1, const cv::Mat pose_2, const cv::Mat K,
                 const PointVector& pts_1, const PointVector& pts_2, Point3Vector& triangulated) {
  //
  // cv::Mat identity = cv::Mat::eye(3, 4, CV_32F);
  cv::Mat proj_1 = cv::Mat_<float>(K * pose_1);
  cv::Mat proj_2 = cv::Mat_<float>(K * pose_2);
  cv::Mat output_pts = cv::Mat(4, pts_2.size(), CV_32F);
  cv::triangulatePoints(proj_1, proj_2, pts_1, pts_2, output_pts);
  // cv::convertPointsFromHomogeneous(output_pts, triangulated);
  convert_from_homogeneous4d(output_pts, triangulated);
}

cv::Mat estimate_motion(const PointVector& pts_1, const PointVector& pts_2, const cv::Mat& F,
                        const cv::Mat& K) {
  /*
    TODO: Points3Vector bootstrap(pts_1, pts_2, F, cv::Mat K)
    F: Fundamental matrix
    K: Camera Intrinsic matrix
  */

  // Essential matrix
  // cv::Mat_<double>(
  cv::Mat E = K.t() * F * K;
  // cv::Mat E = cv::Mat_<double>(K.t()) * cv::Mat_<double>(F) * cv::Mat_<double>(K);

  cv::Mat W = (cv::Mat_<float>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
  cv::Mat Z = (cv::Mat_<float>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 0);
  cv::Mat pose, R, T_skew, t;                     // Motion candidates
  cv::Mat U, S, V_t;                              // SVD
  cv::Mat identity = cv::Mat::eye(3, 4, CV_32F);  // Initial pose

  cv::SVD::compute(E, S, U, V_t);

  if (cv::determinant(U) < 0) {
    U *= -1;
  }
  if (cv::determinant(V_t) < 0) {
    V_t *= -1.0;
  }

  T_skew = U * Z * U.t();
  t = deskew(T_skew);

  int best_count = 0;
  int current_count = 0;
  // cv::Mat best_translation, best_rotation;
  cv::Mat best_pose;
  // Explicitly test for the transform with the best front-ness

  R = U * W.t() * V_t;
  cv::hconcat(R, t, pose);
  current_count = num_in_front(identity, pose, K, pts_1, pts_2);
  // Don't have to check if this is the best
  // best_translation = t;
  // best_rotation = R;
  best_pose = pose.clone();
  best_count = current_count;

  cv::hconcat(R, -t, pose);
  current_count = num_in_front(identity, pose, K, pts_1, pts_2);
  // Any way to avoid repeating this?
  if (current_count > best_count) {
    best_count = current_count;
    // best_rotation = R.clone();
    // best_translation = -t.clone();
    best_pose = pose.clone();
  }

  R = W * V_t;
  cv::hconcat(R, t, pose);
  current_count = num_in_front(identity, pose, K, pts_1, pts_2);
  if (current_count > best_count) {
    best_count = current_count;
    // best_rotation = R.clone();
    // best_translation = t.clone();
    best_pose = pose.clone();
  }

  cv::hconcat(R, -t, pose);
  current_count = num_in_front(identity, pose, K, pts_1, pts_2);
  if (current_count > best_count) {
    best_count = current_count;
    // best_rotation = R.clone();
    // best_translation = -t.clone();
    best_pose = pose.clone();
  }
  // CV_Assert(best_translation.Size()) // Something went wrong
  // CV_Assert(best_rotation.Size()) // Something went wrong
  return best_pose;
}
}