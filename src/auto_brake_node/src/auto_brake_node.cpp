
// #include <ros/ros.h>
//ROS标准msg头文件
// #include <std_msgs/Float32.h>
#include "std_msgs/Float32.h"
#include"ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "macros_generated.h"

/**
 * @brief 主动刹车回调函数。实现利用scan数据进行主动刹车控制，并且发布cmd_vel类型数据
  */
void Auto_Callback(const sensor_msgs::LaserScan &scan)
{  
  
    std_msgs::Float32 distance;
    // ROS_INFO("Scan msg" scan.angle_max,scan.angle_min,scan.intensities");
    // distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
    //float distance = sqrt(pow(msg->x,2)+pow(msg->y,2));
    // ROS_INFO("Listener: Distance to origin = %f, state: %s",distance.data,msg->state.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_brake_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1, Auto_Callback);
  //ros::spin()用于调用所有可触发的回调函数。将进入循环，不会返回，类似于在循环里反复调用ros::spinOnce()。
  ros::spin(); 
  return 0;
}

