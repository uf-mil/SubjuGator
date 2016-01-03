// Interface to PCL to visualize various garbage
// Do we have a method for drawing an ellipse in PCL?
// Crap -- Can't compile PCL w/ C++11
// Damnit...
#include <vector>
#include <memory>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <mgl2/qt.h>
#include <sub8_slam/slam.h>

namespace slam {

class SphereDraw : public mglDraw {
 public:
  Point3Vector data;
  int Draw(mglGraph* gr);
  void set_data(const Point3Vector& point3vector) { data = point3vector; }
};
//-----------------------------------------------------
int SphereDraw::Draw(mglGraph* gr) {
  gr->Rotate(60, 40);
  Point3 point;
  for (size_t i = 0; i < data.size(); i++) {
    point = data[i];
    gr->Sphere(mglPoint(point.x, point.y, point.z), 0.01);
  }
  gr->SetRanges(-5.0, 5.0, -5.0, 5.0, -5.0, 5.0);

  return 0;
}

void visualize_point3vector(const Point3Vector& point3vector) {
  // TODO: Draw a plot and then die
  SphereDraw drawer;
  drawer.set_data(point3vector);
  mglQT gr(&drawer, "Point Vector Visualization");
  gr.Run();
}

// namespace RvizVisualizer {
RvizVisualizer::RvizVisualizer() {
  camera_pub = nh.advertise<visualization_msgs::Marker>("/cameras", 1);
  point_pub = nh.advertise<visualization_msgs::Marker>("/points", 1);
  ros::spinOnce();
  // Give these guys some time to get ready
  ros::Duration(0.5).sleep();
}
void RvizVisualizer::create_marker(visualization_msgs::Marker& marker) {
  visualization_msgs::Marker camera_marker, point_marker;
  camera_marker.header.frame_id = "/map";
  camera_marker.header.stamp = ros::Time::now();
  camera_marker.ns = "map";
  camera_marker.id = 0;
  camera_marker.action = visualization_msgs::Marker::ADD;
  camera_marker.pose.position.x = 0;
  camera_marker.pose.position.y = 0;
  camera_marker.pose.position.z = 0;
  camera_marker.pose.orientation.x = 0.0;
  camera_marker.pose.orientation.y = 0.0;
  camera_marker.pose.orientation.z = 0.0;
  camera_marker.pose.orientation.w = 1.0;
  camera_marker.scale.x = 0.02;
  camera_marker.scale.y = 0.02;
  camera_marker.scale.z = 0.02;
  camera_marker.color.r = 0.0f;
  camera_marker.color.g = 1.0f;
  camera_marker.color.b = 1.0f;
  camera_marker.color.a = 1.0f;
  camera_marker.lifetime = ros::Duration();
  camera_marker.type = visualization_msgs::Marker::LINE_LIST;

  point_marker = camera_marker;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 0.0f;
  point_marker.color.b = 0.0f;
  point_marker.color.a = 1.0f;
  point_marker.scale.x = 0.02;
  point_marker.scale.y = 0.02;
  point_marker.scale.z = 0.02;
  point_marker.type = visualization_msgs::Marker::POINTS;
  marker = point_marker;
}
void RvizVisualizer::draw_points(Point3Vector points) {
  visualization_msgs::Marker point_marker;
  create_marker(point_marker);
  point_marker.points.resize((int)(points.size()));
  point_marker.colors.resize((int)(points.size()));
  Point3 point;
  for (int i = 0; i < points.size(); i++) {
    point = points[i];
    point_marker.colors[i].a = 1.0f;
    point_marker.colors[i].r = 1.0f;
    point_marker.points[i].x = point.x;
    point_marker.points[i].y = -point.y;
    point_marker.points[i].z = -point.z;
  }
  point_pub.publish(point_marker);
}
}