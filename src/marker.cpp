#include "marker.h"

#include <visualization_msgs/Marker.h>

static ros::Publisher randomTree;
static ros::Publisher selectedPath;

void markerInit(ros::NodeHandle& n) {
  randomTree = n.advertise<visualization_msgs::Marker>(
      "random_tree",
      1,
      true);
  selectedPath = n.advertise<visualization_msgs::Marker>(
      "selected_path",
      1,
      true);
}

static int lastId = 1;

void drawLine(int color, vector<Point>& points) {
  double x = 0;
  double y = 0;
  double steps = 50;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = lastId;
  lastId++;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.05;
  lines.color.r = 1.0;
  lines.color.b = 0.5*color;
  lines.color.a = 1.0;

  for (int i = 0; i < points.size(); ++i) {
    lines.points.push_back(points[i]);
  }

  //publish new curve
  if (color == 0) {
    selectedPath.publish(lines);
  } else {
    randomTree.publish(lines);
  }
}
