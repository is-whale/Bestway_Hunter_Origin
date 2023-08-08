#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>

void chatterCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  double x=msg->pose.pose.position.x;
  double y=msg->pose.pose.position.y;
  //double theta=msg->pose.pose.orientation;
  std::cout<<x<<y<<std::endl;
}

void chatterCallback1(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  double x=msg->point.x;
  double y=msg->point.y;
  //double theta=msg->pose.pose.orientation;
  std::cout<<x<<y<<std::endl;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "initialfromrviz");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/initialpose", 1000, chatterCallback);
//  ros::Subscriber sub = nh.subscribe("/clicked_point", 1000, chatterCallback1);
while(ros::ok())
{
  ros::spinOnce();
}
  std::cout<<"........................."<<std::endl;

  return 0;
}