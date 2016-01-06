#include <sub8_slam/slam.h>

namespace slam {

void draw_points(cv::Mat& frame, const PointVector& points, int radius, int thickness) {
  // TODO: Let user pass a color
  for (unsigned int k = 0; k < points.size(); k++) {
    cv::circle(frame, points[k], radius, cv::Scalar(240, 0, 0), thickness);
    // todo: red rectangle outlines would look cooler
  }
}

void draw_point_ids(cv::Mat& frame, const PointVector& points, const IdVector& point_ids) {
  for (unsigned int k = 0; k < points.size(); k++) {
    // cv::circle(frame, points[k], radius, cv::Scalar(240), 1);
    cv::putText(frame, std::to_string(point_ids[k]), points[k], cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 250, 0));
  }
}

void draw_reprojection(cv::Mat& frame, const Point3Vector& points3d, const Pose& pose,
                       const cv::Mat& K) {
  cv::Mat points2d_est;
  // cv::Mat points2d_measured(points2d);
  cv::Mat rotation_vector;
  cv::Rodrigues(pose.rotation, rotation_vector);
  cv::projectPoints(points3d, rotation_vector, pose.translation, K, cv::Mat(), points2d_est);
  draw_points(frame, points2d_est, 2, -1);
}
}