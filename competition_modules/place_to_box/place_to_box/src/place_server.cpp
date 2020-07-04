#include "ros/ros.h"
#include <ros/package.h>
#include "place_to_box/data.h"
#include <stdio.h>

bool recieve_command(place_to_box::data::Request &req, place_to_box::data::Response &res){
  printf("position :\n");
  printf(" x: %f\n", req.pose.position.x);
  printf(" y: %f\n", req.pose.position.y);
  printf(" z: %f\n", req.pose.position.z);
  printf("orientation :\n");
  printf(" x: %f\n", req.pose.orientation.x);
  printf(" y: %f\n", req.pose.orientation.y);
  printf(" z: %f\n", req.pose.orientation.z);
  printf(" w: %f\n", req.pose.orientation.w);
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_command_server");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("place", recieve_command);
  ros::spin();
  return 0;
}
