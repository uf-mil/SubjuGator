/*

  "Frame" class
    x --> Has features associated with it
    x --> Some kind of graph of features that exist in the next frame
      --> (Perhaps only *detect* new features in new keyframes)
    --> Distinguish between normal frames and keyframes
    --> local sba requires all frames
    --> global SBA requires all frames
    --> global SBA refinement requires only keyframes

  --> Bootstrap
    x --> LK
    x --> Triangulate
    --> SBA

  --> Tracking
    --> Initialize
      x --> Optical Flow
      x --> Detect new points
      x --> Track them & correspondence
      --> Depth filter them
      --> Covariance weighted epipolar search

  --> Estimate Motion
    x --> Estimate fundamental matrix
    x --> Triangulate points
    --> Improve with SBA
      --> SBA tools
    --> PnP solving
      --> Huber distance Lie-Alg optimization?
      --> GN solver
        --> Inverse Compositional solver
    --> Bayes Filter
      --> Discrete/histogram
      --> Parametric mixture-model
      --> OpenCL histogram
      --> Particle filter feasible?
      --> Unscented transform feasible?

    --> Reinitialize when needed
      --> Not restricting ourselves to pure optical flow may...
        - Improve speed (only search on epipolar lines + covariance)
        - Increase robustness to sudden motion
      --> Kalman filter pose estimate (This may not be necessary)

  --> Loop closure
    --> Place recognition
    --> g2o (A little more complicated, but more globally useful than sba.h)
    --> Sparse bundle-adjustment
      x --> Get SBA to build
      --> Make some crappy SBA classes

  --> Visualization
    x --> Visualize triangulated points
    x --> Consider using RVIZ to visualize
        x --> Make visualizer for our own triangulation
        x --> Make Rviz interface
    --> Visualize depth certainty

  --> Testing
    x --> Make an analysis set
    x --> Record data
    --> Make a two-frame-sandbox executable for testing various things

  --> Improvements
    --> SSE on color
    --> Determine when bootstrapping has failed
      (Scale invariant minimum motion requirement)
      (Minimum sum translation?)
      --> Struggle depth certainty is probably an indicator of a need for reinitializing

  --> Experiments
    --> See if 32F improves speed...it will not affect accuracy

  --> Try OpenCL for...
    --> Optical Flow
    --> Explicit posterior computation
    --> SSE for larger patches (invariance promises weaken past 4x4)
      --> pose to SSE

  x --> Put this in Sub Git
*/

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sub8_slam/slam.h>
#include "ros/ros.h"

// Probably not super smooth to initialize using normal arrays
float _intrinsics[] = {978.6015701, 0., 470.27648376, 0., 977.91377625, 270.84941812, 0., 0., 1.};
float _distortion[] = {-4.37883322e-02, +1.41220879e+00, +1.12558296e-03, -1.78899693e-03,
                       -5.54287009e+00};

cv::Mat intrinsics(3, 3, CV_32F, _intrinsics);
cv::Mat distortion(5, 1, CV_32F, _distortion);

int main(int argc, char **argv) {
  ros::init(argc, argv, "monocular_slam");
  slam::RvizVisualizer rviz;
  if (argc != 2) {
    std::cout << "Give me a video..." << std::endl;
    return -1;
  }

  const std::string video_source = argv[1];
  cv::VideoCapture cap(video_source);

  if (!cap.isOpened()) {
    std::cout << "Could not open target video " << video_source << std::endl;
    return -1;
  }

  cv::Size refS =
      cv::Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH), (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
  cv::namedWindow("input", CV_WINDOW_AUTOSIZE);
  cvMoveWindow("input", 400, 0);

  slam::IdVector prev_feature_ids, new_feature_ids;
  slam::PointVector prev_feature_locations;
  slam::PointVector new_feature_locations;

  cv::Mat input_frame, new_frame, render_frame;
  cv::Mat prev_frame;
  slam::Pose prev_pose;
  prev_pose.rotation = cv::Mat::eye(3, 3, CV_32F);
  prev_pose.translation = (cv::Mat_<float>(3, 1) << 0.0, 0.0, 0.0);

  int frame_num = 0;
  char key_press = '\n';
  for (;;) {
    //////// Bookkeeping
    cap >> input_frame;
    if (input_frame.empty()) {
      std::cout << "Done";
      break;
    }

    /////// Preprocessing
    slam::preprocess(input_frame, new_frame, intrinsics, distortion);

    ////// Initialization
    if (prev_feature_locations.size() == 0) {
      std::cout << "Initializing Frame\n";
      slam::initialize(new_frame, prev_feature_locations, prev_feature_ids);
      // Initialize previous feature ids to a standard range
      prev_frame = new_frame.clone();
      continue;
    }

    ////// Track known points and dismiss outliers
    slam::StatusVector status;
    slam::optical_flow(prev_frame, new_frame, prev_feature_locations, new_feature_locations, status);
    // Determine the IDs of the preserved points
    // TODO: Filter and which_pts in the same function
    new_feature_ids = slam::which_points(status, prev_feature_ids);
    new_feature_locations = slam::filter(status, new_feature_locations);
    prev_feature_locations = slam::filter(status, prev_feature_locations);

    ////// Compute the fundametnal matrix and dismiss outliers
    slam::StatusVector inliers;
    cv::Mat F;
    F = slam::estimate_fundamental_matrix(prev_feature_locations, new_feature_locations, inliers);
    new_feature_ids = slam::which_points(inliers, new_feature_ids);
    // Filter by inlier mask
    new_feature_locations = slam::filter(inliers, new_feature_locations);
    prev_feature_locations = slam::filter(inliers, prev_feature_locations);

    ////// Estimate motin from the fundamental matrix
    slam::Pose pose_T;
    pose_T = slam::estimate_motion(prev_feature_locations, new_feature_locations, F, intrinsics);
    std::cout << pose_T.rotation << 't' << std::endl;

    ////// Triangulate points and dismiss the guess based on magnitude of reprojection error
    slam::Point3Vector triangulated;
    slam::triangulate(prev_pose, pose_T, intrinsics, prev_feature_locations, new_feature_locations, triangulated);
    double error_amt = slam::average_reprojection_error(triangulated, new_feature_locations, pose_T, intrinsics);
    std::cout << "Error: " << error_amt << std::endl;

    ////// Initialize a slam frame based on these shenanigans
    if (error_amt < 100.0) {
      slam::Frame(pose_T, new_feature_ids, new_feature_locations);
    }

    // Make a nice new frame, on which to draw colored things
    cv::cvtColor(new_frame, render_frame, CV_GRAY2BGR, 3);
    slam::draw_points(render_frame, new_feature_locations);
    slam::draw_point_ids(render_frame, new_feature_locations, new_feature_ids);
    rviz.draw_points(triangulated, error_amt > 100.0);

    prev_feature_locations = new_feature_locations;
    prev_frame = new_frame.clone();
    prev_feature_ids = new_feature_ids;

    ++frame_num;
    std::cout << "Frame: " << frame_num << "\n";
    cv::imshow("input", render_frame);
    std::cout << std::endl;
    key_press = (char)cv::waitKey(50);
    if (key_press == 'q') break;
  }

  return 0;
}