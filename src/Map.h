#pragma once

#include <vector>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace geometry_msgs;

class Map {
 public:
  Map(const nav_msgs::OccupancyGrid& msg);
  bool isOccupied(float x, float y);

  void drawPositions();

  bool robotAreaOccupied(Pose robotPose);

 private:
  unsigned int mWidth, mHeight;
  float mResolution;
  vector<vector<bool>* > mMap;
  Pose mPose;

  bool isCellIndexOccupied(unsigned int xGrid, unsigned int yGrid);
};
