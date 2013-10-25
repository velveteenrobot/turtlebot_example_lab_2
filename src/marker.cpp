#include "marker.h"

#include <visualization_msgs/Marker.h>

static ros::Publisher marker_pub;

void markerInit(ros::NodeHandle& n) {
  marker_pub = n.advertise<visualization_msgs::Marker>(
      "visualization_marker",
      1,
      true);
}

void drawPoint(float x, float y) {
  static int lastId = 1;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = lastId; //each curve must have a unique id or you will overwrite an old ones
  lastId++;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.1;
  lines.color.r = 1.0;
  lines.color.b = 0.2;
  lines.color.a = 1.0;

  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = 0; //not used
  lines.points.push_back(p);
  p.x = x + 0.1;
  p.y = y + 0.1;
  p.z = 0; //not used
  lines.points.push_back(p);

  //publish new curve
  marker_pub.publish(lines);
}

//Example of drawing a curve
void drawCurve(int k) {
  // Curves are drawn as a series of stright lines
  // Simply sample your curves into a series of points

  double x = 0;
  double y = 0;
  double steps = 50;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = k; //each curve must have a unique id or you will overwrite an old ones
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.1;
  lines.color.r = 1.0;
  lines.color.b = 0.2*k;
  lines.color.a = 1.0;

  //generate curve points
  for(int i = 0; i < steps; i++) {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0; //not used
    lines.points.push_back(p);

    //curve model
    x = x+0.1;
    y = sin(0.1*i*k);
  }

  //publish new curve
  marker_pub.publish(lines);

}
