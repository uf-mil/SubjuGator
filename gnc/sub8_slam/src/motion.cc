#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sub8_slam/slam.h>

namespace slam {

cv::Mat estimate_fundamental_matrix(const PointVector& pts_1, const PointVector& pts_2,
                                    StatusVector& inliers) {
  cv::Mat F;
  F = cv::findFundamentalMat(pts_1, pts_2, cv::FM_RANSAC, 2, 0.999, inliers);
  return cv::Mat_<float>(F);
}

int num_in_front(const cv::Mat pose_1, const cv::Mat pose_2, const cv::Mat K,
                 const PointVector& pts_1, const PointVector& pts_2) {
  /*
    Compute the number of points in front, without dehomogenizing
  */
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
  // Opencv convert had very undesirable behavior
  for (unsigned int k; k < src.cols; k++) {
    cv::Mat row = src.col(k);
    output.push_back(Point3(row.at<float>(0) / row.at<float>(3),
                            row.at<float>(1) / row.at<float>(3),
                            row.at<float>(2) / row.at<float>(3)));
  }
}

cv::Mat deskew(const cv::Mat T) {
  cv::Mat t = (cv::Mat_<float>(3, 1) << T.at<float>(2, 1), T.at<float>(0, 2), T.at<float>(1, 0));
  return t;
}

void triangulate(const Pose& pose_1, const Pose& pose_2, const cv::Mat K, const PointVector& pts_1,
                 const PointVector& pts_2, Point3Vector& triangulated) {
  cv::Mat mat_pose_1, mat_pose_2;
  cv::hconcat(pose_1.rotation, pose_1.translation, mat_pose_1);
  cv::hconcat(pose_2.rotation, pose_2.translation, mat_pose_2);
  cv::Mat proj_1 = cv::Mat_<float>(K * mat_pose_1);
  cv::Mat proj_2 = cv::Mat_<float>(K * mat_pose_2);
  cv::Mat output_pts = cv::Mat(4, pts_2.size(), CV_32F);
  cv::triangulatePoints(proj_1, proj_2, pts_1, pts_2, output_pts);
  convert_from_homogeneous4d(output_pts, triangulated);
}

Pose estimate_motion_pnp(const Point3Vector& pts_3d, const PointVector& pts_2d, const cv::Mat& K) {
  // TODO: Permit extrinsic guess
  // cv::solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix,
  //             InputArray distCoeffs, OutputArray rvec, OutputArray tvec,
  //             bool useExtrinsicGuess = false, int flags = ITERATIVE);
  cv::Mat translation_vector;
  cv::Mat rotation_vector;
  cv::solvePnP(pts_3d, pts_2d, K, cv::Mat(), rotation_vector, translation_vector, false,
               CV_ITERATIVE);
  Pose pose;
  pose.translation = translation_vector;
  // How bout that!
  cv::Rodrigues(rotation_vector, pose.rotation);
  // Quick-inverse
  pose.rotation = -pose.rotation.t();  // Transpose R
  pose.translation = pose.rotation * pose.translation;
  return pose;
}

Pose estimate_motion_fundamental_matrix(const PointVector& pts_1, const PointVector& pts_2,
                                        const cv::Mat& F, const cv::Mat& K) {
  /*
    This + triangulation is the bootstrapping I talk so much about.
      F: Fundamental matrix
      K: Camera Intrinsic matrix
    TODO: Points3Vector bootstrap(pts_1, pts_2, F, cv::Mat K)

    The pose is in the Sim(3) frame of the first camera pose, with \lambda being close to the
      distance between the cameras in the two frames
  */

  // Essential matrix
  cv::Mat E = K.t() * F * K;

  cv::Mat W = (cv::Mat_<float>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
  cv::Mat Z = (cv::Mat_<float>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 0);
  cv::Mat pose, R1, R2, T_skew, t;                // Motion candidates
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
  Pose best_pose;

  // Explicitly test for the transform with the best front-ness
  // This will be called a handful of times, and does not merit serious optimization
  R1 = U * W.t() * V_t;
  cv::hconcat(R1, t, pose);
  current_count = num_in_front(identity, pose, K, pts_1, pts_2);
  // Don't have to check if this is the best
  best_pose.rotation = R1;
  best_pose.translation = t;
  best_count = current_count;

  cv::hconcat(R1, -t, pose);
  current_count = num_in_front(identity, pose, K, pts_1, pts_2);
  // Any way to avoid repeating this?
  if (current_count > best_count) {
    best_count = current_count;
    best_pose.rotation = R1;
    best_pose.translation = -t;
  }

  R2 = W * V_t;
  cv::hconcat(R2, t, pose);
  current_count = num_in_front(identity, pose, K, pts_1, pts_2);
  if (current_count > best_count) {
    best_count = current_count;
    best_pose.rotation = R2;
    best_pose.translation = t;
  }

  cv::hconcat(R2, -t, pose);
  current_count = num_in_front(identity, pose, K, pts_1, pts_2);
  if (current_count > best_count) {
    best_count = current_count;
    best_pose.rotation = R2;
    best_pose.translation = -t;
  }
  return best_pose;
}

double average_reprojection_error(const Point3Vector& points3d, const PointVector& points2d,
                                  const Pose& pose, const cv::Mat& K) {
  cv::Mat points2d_est;
  cv::Mat points2d_measured(points2d);
  cv::Mat error;
  double average_error;

  cv::Mat rotation_vector;
  cv::Rodrigues(pose.rotation, rotation_vector);
  cv::projectPoints(points3d, rotation_vector, pose.translation, K, cv::Mat(), points2d_est);
  error = points2d_est - points2d_measured;
  for (unsigned int k; k < error.cols; k++) {
    average_error += cv::norm(error.col(k));
  }
  average_error /= error.cols;
  return average_error;
}
}