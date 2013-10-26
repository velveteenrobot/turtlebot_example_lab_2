#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>

using namespace std;
using namespace geometry_msgs;

void markerInit(ros::NodeHandle& n);
void drawPoint(float x, float y);
void drawLine(int color, vector<Point>& points);
