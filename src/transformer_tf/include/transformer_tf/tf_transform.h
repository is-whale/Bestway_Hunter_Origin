#ifndef TF_TRANSFORM_H
#define TF_TRANSFORM_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

class transformer_tf
{
    public:
        transformer_tf();
        ~transformer_tf();
        void init();
        void run();
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        std::string lidar_frame_;
        std::string map_frame_;
        std::string odom_frame_;
        std::string base_link_frame_;
        std::string base_foot_frame_;
        ros::Subscriber sub_laserscan_;
        ros::Subscriber sub_odom_;
        static tf2_ros::TransformBroadcaster broadcaster;

        void OdomTFMapCallback(const nav_msgs::Odometry& odom_msg);
        void LaserTFBaseCallback(const sensor_msgs::LaserScan& laserscan_msg);
};

#endif