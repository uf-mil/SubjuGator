#include <sub8_perception/navigation_gate.hpp>

#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_gate");
  ROS_INFO("Initializing node /navigation_gate");
  boost::shared_ptr<Sub8NavigationGateDetector> sub8_navigation_gates(new Sub8NavigationGateDetector());
  ROS_INFO("Now Accepting perception service calls");
  ros::spin();
}
