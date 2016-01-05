#include <sub8_slam/slam.h>

namespace slam {

void draw_points(cv::Mat& frame, const PointVector& points) {
  int radius = 5;
  for (unsigned int k = 0; k < points.size(); k++) {
    cv::circle(frame, points[k], radius, cv::Scalar(240, 0, 0), 1);
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

}