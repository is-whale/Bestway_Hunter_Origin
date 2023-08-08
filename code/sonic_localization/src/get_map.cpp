#include <ros/ros.h> 
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <string>
#include <std_msgs/Int16.h>
#include "common.h"



int main (int argc, char **argv) 
{ 
	ros::init (argc, argv, "get_map"); 
	
	ros::NodeHandle nh; 

	ros::Publisher surf_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_cloud_surf", 1);
	sensor_msgs::PointCloud2 surf_msg; 
	ros::Publisher corner_pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_cloud_corner", 1);
	sensor_msgs::PointCloud2 corner_msg;
          
	
    //从目标文件读取pcd，到cloud surf corner
    pcl::PointCloud<PointType>::Ptr  surf_map (new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr  corner_map (new pcl::PointCloud<PointType>);
	pcl::io::loadPCDFile<PointType>(argv[1], *surf_map);
	pcl::io::loadPCDFile<PointType>(argv[2], *corner_map);
 
	if (pcl::io::loadPCDFile<PointType>(argv[1], *surf_map) == -1)
	{
		std::cout << "surf cloud reading failed." << std::endl;
		return (-1);
	}

	if (pcl::io::loadPCDFile<PointType>(argv[2], *corner_map) == -1)
	{
		std::cout << "corner cloud reading failed." << std::endl;
		return (-1);
	}
	// Fill in the cloud data 
	for(int i = 0 ;i<surf_map->points.size();i++){
		float tmp = surf_map->points[i].x;
		surf_map->points[i].x = surf_map->points[i].z;
		surf_map->points[i].z = surf_map->points[i].y;
		surf_map->points[i].y = tmp;
	}

	for(int i = 0 ;i<corner_map->points.size();i++){
		float tmp = corner_map->points[i].x;
		corner_map->points[i].x = corner_map->points[i].z;
		corner_map->points[i].z = corner_map->points[i].y;
		corner_map->points[i].y = tmp;
	}

	//Convert the cloud to ROS message 
	pcl::toROSMsg(*surf_map, surf_msg); 
	surf_msg.header.frame_id = "/camera_init"; 
	
	pcl::toROSMsg(*corner_map, corner_msg); 
	corner_msg.header.frame_id = "/camera_init"; 
	
	ros::Rate loop_rate(1); 
	while (ros::ok()) 
	{ 
		surf_pub.publish(surf_msg);
		corner_pub.publish(corner_msg);
		loop_rate.sleep(); 
	} 
	return 0; 
}