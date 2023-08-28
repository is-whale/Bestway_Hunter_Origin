#include<ros/ros.h>

int main (int argc,int argv** )
{
    ros::init(argc,argv,"auto_brake_node");
    ros::NodeHandle private_nh("~");
    
    ros::spin();//循环读取接收的数据，并调用回调函数处理
}