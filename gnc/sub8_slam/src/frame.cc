#include <sub8_slam/slam.h>

namespace slam {

Frame::Frame(cv::Mat& image, Pose& pose, IdVector& feature_ids, PointVector& feature_locations) {
  set_image(image);  // Hmm...
  set_features(feature_ids, feature_locations);
  set_pose(pose);
}

Frame::Frame(Pose& pose, IdVector& feature_ids, PointVector& feature_locations) {
  set_features(feature_ids, feature_locations);
  set_pose(pose);
}

void Frame::set_pose(Pose& pose) {
  // TODO This is super non-optimized, we can probably get away with avoiding clones
  camera_pose.translation = pose.translation.clone();
  camera_pose.rotation = pose.rotation.clone();
}

void Frame::set_features(IdVector& feature_ids, PointVector& feature_locations) {
  // TODO Super non-optimized, probably can avoid so many copies
  // TODO: Verify that this actually copies
  // vector::operator= copies
  this->feature_ids = feature_ids;
  this->feature_locations = feature_locations;
}

void Frame::set_image(cv::Mat& image) {
  this->image = image.clone();
}
}