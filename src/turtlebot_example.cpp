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

void pose_callback(const turtlebot_example::ips_msg& msg)
{
  //This function is called when a new position message is received
  if(msg.tag_id != TAGID) {
    return;
  }

  double X = msg.X; // Robot X psotition
  double Y = msg.Y; // Robot Y psotition
  double Yaw = msg.Yaw; // Robot Yaw

  // std::cout << "X: " << X << ", Y: " << Y << ", Yaw: " << Yaw << std::endl ;
}


//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
  //This function is called when a new map is received
  //you probably want to save the map into a form which is easy to work with

  Map map(msg);
  map.drawPositions();
  // TODO: calculate the path
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
  ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>(
      "/cmd_vel_mux/input/navi",
      1);
  markerInit(n);

  //Velocity control variable
  geometry_msgs::Twist vel;

  //Set the loop rate
  ros::Rate loop_rate(1/CYCLE_TIME);    //20Hz update rate

  while (ros::ok())
  {
    loop_rate.sleep(); //Maintain the loop rate
    ros::spinOnce();   //Check for new messages

    //Main loop code goes here:
    // vel.linear.x = 0.1; // set linear speed
    // vel.angular.z = 0.3; // set angular speed

    velocity_publisher.publish(vel); // Publish the command velocity
  }

  return 0;
}
