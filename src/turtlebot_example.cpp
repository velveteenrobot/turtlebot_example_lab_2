//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include "turtlebot_example.h"
#include "Map.h"
#include "marker.h"
#include "RRT.h"
#include "tracking.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <turtlebot_example/ips_msg.h>

#include <vector>

using namespace std;

//Callback function for the Position topic (LIVE)

static Map* roomMap = NULL;
static bool poseReady = false;
static Pose pose;


void pose_callback(const turtlebot_example::ips_msg& msg)
{
  //This function is called when a new position message is received
  if(msg.tag_id != TAGID) {
    return;
  }

  pose.position.x = msg.X;
  pose.position.y = msg.Y;

  quaternionTFToMsg(
      tf::createQuaternionFromRPY(0, 0, msg.Yaw),
      pose.orientation);
  poseReady = true;
}


//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
  //This function is called when a new map is received
  //you probably want to save the map into a form which is easy to work with

  roomMap = new Map(msg);
  // TODO: calculate the path
}

void spinOnce(ros::Rate& loopRate) {
  loopRate.sleep(); //Maintain the loop rate
  ros::spinOnce();   //Check for new messages
}

int main(int argc, char **argv)
{
  //Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;

  //Subscribe to the desired topics and assign callbacks
  ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

  //Setup topics to Publish from this node
  markerInit(n);

  //Set the loop rate
  ros::Rate loopRate(1/CYCLE_TIME);    //20Hz update rate

  // wait for the position
  while (!poseReady) {
    spinOnce(loopRate);
  }
  // wiat for the map
  while (roomMap == NULL) {
    spinOnce(loopRate);
  }

  // plan a path
  cout<<"Running RRT"<<endl;
  vector<Pose> waypoints;
  Pose dest;
  dest.position.x = 6;
  dest.position.y = 3;
  waypoints.push_back(dest);
  dest.position.x = 6;
  dest.position.y = -3;
  waypoints.push_back(dest);

  list<Milestone*> path = doRRTWaypoints(pose, waypoints, *roomMap);
  /*
  list<Milestone*> path;
  Pose unusedPose;
  path.push_back(new Milestone(NULL, unusedPose, 0.2, 0, 100));
  */
  // track the path
  Tracking tracking(path, pose, n);
  while (ros::ok()) {
    //Velocity control variable
    if (!tracking.doCycle(pose)) {
      break;
    }
    spinOnce(loopRate);
  }
  // TODO: free memory
  return 0;
}
