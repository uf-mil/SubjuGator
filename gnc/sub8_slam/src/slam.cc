/*

  "Frame" class
    --> Has features associated with it
    --> Some kind of graph of features that exist in the next frame
      --> (Perhaps only *detect* new features in new keyframes)

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
    --> Do Bayes shenanigans
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
    --> Visualize depth certainty
    --> Consider using RVIZ to visualize

  --> Improvements
    --> SSE on color
    --> Determine when bootstrapping has failed
      (Scale invariant minimum motion requirement)
      (Minimum sum translation?)

  --> Experiments
    --> See if 32F improves speed...it will not affect accuracy

  --> Try OpenCL for...
    --> Optical Flow
    --> Explicit posterior computation
    --> SSE for larger patches (invariance promises weaken past 4x4)

  --> Put this in Sub Git
    -->...Or my own git for an eventual merge?
  --> Get CPPCheck
*/

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sub8_slam/slam.h>

// Probably not super smooth to initialize using normal arrays
float _intrinsics[] = {978.6015701, 0., 470.27648376, 0., 977.91377625, 270.84941812, 0., 0., 1.};
float _distortion[] = {-4.37883322e-02, +1.41220879e+00, +1.12558296e-03, -1.78899693e-03,
                       -5.54287009e+00};

cv::Mat intrinsics(3, 3, CV_32F, _intrinsics);
cv::Mat distortion(5, 1, CV_32F, _distortion);

int main(int argc, char* argv[]) {
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

  std::vector<int> previous_feature_ids;
  slam::PointVector prev_pts;
  slam::PointVector new_pts;

  cv::Mat input_frame, current_frame, render_frame;
  cv::Mat previous_frame;
  cv::Mat prev_pose = cv::Mat::eye(3, 4, CV_32F);

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
    slam::preprocess(input_frame, current_frame, intrinsics, distortion);

    ////// Initialization
    if (prev_pts.size() == 0) {
      std::cout << "Initializing Frame\n";
      slam::initialize(current_frame, prev_pts);
      // Initialize previous feature ids to a standard range
      std::iota(previous_feature_ids.begin(), previous_feature_ids.end(), 0);
      previous_frame = current_frame.clone();
      continue;
    }

    std::vector<uchar> status;
    slam::optical_flow(previous_frame, current_frame, prev_pts, new_pts, status);
    new_pts = slam::filter(status, new_pts);
    prev_pts = slam::filter(status, prev_pts);

    std::vector<uchar> inliers;
    cv::Mat F;
    F = slam::estimate_fundamental_matrix(prev_pts, new_pts, inliers);
    // Filter by inlier mask
    new_pts = slam::filter(inliers, new_pts);
    prev_pts = slam::filter(inliers, prev_pts);

    cv::Mat pose_T;
    pose_T = slam::estimate_motion(prev_pts, new_pts, F, intrinsics);
    std::cout << pose_T << 't' << std::endl;

    slam::Point3Vector triangulated;
    slam::triangulate(prev_pose, pose_T, intrinsics, prev_pts, new_pts, triangulated);
    std::cout << "TTT " << triangulated.size() << std::endl;
    slam::visualize_point3vector(triangulated);

    render_frame = current_frame.clone();
    slam::draw_points(render_frame, new_pts);

    prev_pts = new_pts;
    previous_frame = current_frame.clone();

    ++frame_num;
    std::cout << "Frame: " << frame_num << "\n";
    cv::imshow("input", render_frame);
    std::cout << std::endl;
    key_press = (char)cv::waitKey(50);
    if (key_press == 'q') break;
  }

  return 0;
}