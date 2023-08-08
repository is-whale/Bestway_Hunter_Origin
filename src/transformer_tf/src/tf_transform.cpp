#include "transformer_tf/tf_transform.h"

transformer_tf::transformer_tf()
    :private_nh_("~")
{
}

transformer_tf::~transformer_tf()
{
}

void transformer_tf::init()
{
    private_nh_.param<std::string>("odom_frame",odom_frame_, "odom");
    private_nh_.param<std::string>("base_foot_frame",base_foot_frame_, "base_footprint");
    ROS_INFO("init");

}

void transformer_tf::run()
{
    sub_odom_ = nh_.subscribe("/odom", 1, &transformer_tf::OdomTFMapCallback, this);
    ROS_INFO("run");
}

void transformer_tf::OdomTFMapCallback(const nav_msgs::Odometry& odom_msg)
{
    static tf2_ros::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = odom_frame_;
    tfs.child_frame_id = base_foot_frame_;
    tfs.transform.translation.x = odom_msg.pose.pose.position.x;
    tfs.transform.translation.y = odom_msg.pose.pose.position.y;
    tfs.transform.translation.z = odom_msg.pose.pose.position.z;
    tfs.transform.rotation.x = odom_msg.pose.pose.orientation.x;
    tfs.transform.rotation.y = odom_msg.pose.pose.orientation.y;
    tfs.transform.rotation.z = odom_msg.pose.pose.orientation.z;
    tfs.transform.rotation.w = odom_msg.pose.pose.orientation.w;
    broadcaster.sendTransform(tfs);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_transform");
  transformer_tf tf_transform;
  tf_transform.init();
  tf_transform.run();
  ros::spin();

  return 0;
}
