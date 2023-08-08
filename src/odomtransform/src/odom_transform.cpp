#include "ros/ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include <nav_msgs/Odometry.h>

void doPose(const nav_msgs::Odometry& pose){
    //  5-1.创建 TF 广播器
    ROS_INFO("EXECUTR THIS!");
    static tf2_ros::TransformBroadcaster broadcaster;
    //  5-2.创建 广播的数据(通过 pose 设置)
    geometry_msgs::TransformStamped tfs2;
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    
    tfs2.header.frame_id = "map";
    tfs2.header.stamp = pose.header.stamp;

    //  |----坐标系 ID
    tfs2.child_frame_id = "odom";

    //  |----坐标系相对信息设置
    tfs2.transform.translation.x = 0.0;
    tfs2.transform.translation.y = 0.0;
    tfs2.transform.translation.z = 0.0; // 二维实现，pose 中没有z，z 是 0
    //  |--------- 四元数设置
    
   tfs2.transform.rotation.x = qtn.getX();
   tfs2.transform.rotation.y = qtn.getY();
   tfs2.transform.rotation.z = qtn.getZ();
   tfs2.transform.rotation.w = qtn.getW();
    broadcaster.sendTransform(tfs2);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"odom_map_fake");
    // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建订阅对象
    ros::Subscriber sub = nh.subscribe("/odom",50,doPose);
    //     5.回调函数处理订阅到的数据(实现TF广播)
    //        
    // 6.spin
    ros::spin();
    return 0;
}
