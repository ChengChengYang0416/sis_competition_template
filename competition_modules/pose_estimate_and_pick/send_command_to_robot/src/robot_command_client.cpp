#include "ros/ros.h"
#include "send_command_to_robot/data.h"
#include <cstdlib>

geometry_msgs::Pose command;
void command_cb(const geometry_msgs::Pose::ConstPtr& msg){
    command = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_command_client");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<send_command_to_robot::data>\
      ("data");

  ros::Subscriber command_sub = nh.subscribe<geometry_msgs::Pose>
              ("/robot_command", 10, command_cb);

  send_command_to_robot::data srv;

  /*
  srv.request.pose.position.x = command.position.x;
  srv.request.pose.position.y = command.position.y;
  srv.request.pose.position.z = command.position.z;
  srv.request.pose.orientation.x = command.orientation.x;
  srv.request.pose.orientation.y = command.orientation.y;
  srv.request.pose.orientation.z = command.orientation.z;
  srv.request.pose.orientation.w = command.orientation.w;
  */

  srv.request.pose.position.x = 0.3;
  srv.request.pose.position.y = 0.0;
  srv.request.pose.position.z = 0.1;
  srv.request.pose.orientation.x = 0;
  srv.request.pose.orientation.y = 0.707;
  srv.request.pose.orientation.z = 0;
  srv.request.pose.orientation.w = 0.707;
  srv.request.id = 0;
  /*
  while (ros::ok()){
    client.call(srv);
    ros::spinOnce();
    rate.sleep();
  }
  */

  client.call(srv);
  ros::spinOnce();

  return 0;
}
