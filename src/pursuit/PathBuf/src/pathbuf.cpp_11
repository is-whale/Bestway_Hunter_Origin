#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <math.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/OccupancyGrid.h>
#define MAXCNT 100 

using namespace std;

ros::Publisher pathpub;
ros::Publisher eposepub_;
ros::Publisher fposepub_;
ros::Publisher signpub_;
int flag = 0;
int pointNum  = 0;
double recal_dis = 1.0;
double curr_dis = 0;
int cnt = 0;
geometry_msgs::PoseStamped curr_pose;
geometry_msgs::PoseStamped curr_target;
nav_msgs::Path curr_path;
geometry_msgs::Pose epose;
geometry_msgs::Pose fpose;
geometry_msgs::PoseStamped eposep;
geometry_msgs::PoseStamped fposep;
std::vector<nav_msgs::Path> curr_paths;
nav_msgs::Path curr_cut;
void odomCallback(const nav_msgs::Odometry &odominfo)
{
    // ROS_INFO("1");
    static tf2_ros::Buffer buf;
    static tf2_ros::TransformListener tl(buf);
    // ROS_INFO("2");
    geometry_msgs::Pose cpose = odominfo.pose.pose;
    geometry_msgs::PoseStamped currpose ;
    geometry_msgs::PoseStamped newpose ;
    currpose.header.frame_id = "odom";
    currpose.pose = cpose;
    try
    {
        newpose = buf.transform(currpose,"map");
        ROS_INFO("TRANSFORM SUCC!");

    }
    catch(const std::exception& e)
    {
        ROS_INFO("error %s",e.what());
    }

    curr_pose = newpose;
    cnt = cnt + 1;
    ROS_INFO("cnt : %d",cnt);
}
double get_dis(const geometry_msgs::Pose p1,geometry_msgs::Pose p2)
{
    return sqrt(pow((p1.position.x-p2.position.x),2)+pow((p1.position.y-p2.position.y),2));
}
int cal_direction(nav_msgs::Path path)
{
    if(path.poses[0].pose.position.x >= path.poses[1].pose.position.x)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}
std::vector<nav_msgs::Path> pathcut(nav_msgs::Path origin_path)
{
    int r_index,l_index = 0;
    nav_msgs::Path cpa;
    cpa.header = origin_path.header;
    cpa.poses.push_back(origin_path.poses[0]);
    std::vector<nav_msgs::Path> cutted_paths;
    ROS_INFO("start cutting:");
    for(r_index = 1;r_index < origin_path.poses.size()-1;r_index++)
    {
        double oa_x = origin_path.poses[r_index].pose.position.x -
            origin_path.poses[r_index - 1].pose.position.x;
        double oa_y = origin_path.poses[r_index].pose.position.y -
            origin_path.poses[r_index - 1].pose.position.y;
        double ab_x = origin_path.poses[r_index + 1].pose.position.x -
            origin_path.poses[r_index].pose.position.x;
        double ab_y = origin_path.poses[r_index + 1].pose.position.y -
            origin_path.poses[r_index].pose.position.y;

    /* Checking for the existance of cusp, in the path, using the dot product
    and determine it's distance from the robot. If there is no cusp in the path,
    then just determine the distance to the goal location. */
        if ( (oa_x * ab_x) + (oa_y * ab_y) < 0.0) 
        {
            if(!cpa.poses.empty() && cpa.poses.size()>2)
                cutted_paths.push_back(cpa);
            cpa.poses.clear();
        }
        else
        {

            cpa.poses.push_back(origin_path.poses[r_index]);
        }
    }
    if(!cpa.poses.empty() && cpa.poses.size()>2)
    {
        cutted_paths.push_back(cpa);
    }
    // ROS_INFO("total len: %ld",cutted_paths.size());
    // for(int ii=0;ii<cutted_paths.size();ii++)
    // {
    //     if(cutted_paths[ii].poses.size()<2)
    //     {
    //         cutted_paths.erase(cutted_paths.begin()+ii);
            
    //         ROS_INFO("erase:%d",ii);
    //         ii=0;
    //     }
    //     else
    //     {
    //         ROS_INFO("current cut %d :%ld",ii,cutted_paths[ii].poses.size());
    //     }
    // }
    // ROS_INFO("total len: %ld",cutted_paths.size());
    return cutted_paths;
}
void pathCallback(const nav_msgs::Path &msg) {
    ROS_INFO("4");
    pointNum = msg.poses.size()-1;
    ROS_INFO("5");
    std::vector<nav_msgs::Path> paths;
    std_msgs::Int32 sign ;
    sign.data = 1;
    try
    {
        
        if(flag==0 || cnt >= MAXCNT)
        {
            ROS_INFO("6");
            curr_path = msg;
            curr_paths = pathcut(curr_path);
            ROS_INFO("7");
            // ROS_INFO("%ld",msg.poses.size());
            // ROS_INFO("%ld",curr_paths.size());
            curr_cut = curr_paths[0];
            ROS_INFO("current cut:%ld",curr_cut.poses.size());
            ROS_INFO("current paths:%ld",curr_paths.size());
            ROS_INFO("9");
            epose = curr_cut.poses[0].pose;//路径起始位置
            fpose = curr_cut.poses[curr_cut.poses.size()-1].pose;//目标位置
            eposep.header.frame_id="map";
            eposep.pose=epose;
            fposep.header.frame_id="map";
            fposep.pose=fpose;
            sign.data = cal_direction(curr_cut);
            ROS_INFO("8");
            eposepub_.publish(eposep);
            fposepub_.publish(fposep);
            pathpub.publish(curr_cut);
            signpub_.publish(sign);
            ROS_INFO("PUBLISH SUCC!");
            cnt = 0;
            flag=1;
        }
        else{
            ROS_INFO("11");
            curr_dis = get_dis(curr_pose.pose,fpose);
            if(curr_dis <= recal_dis)
            {
                ROS_INFO("12");
                curr_paths.erase(curr_paths.begin());
                ROS_INFO("13");
                if(!curr_paths.empty())
                {
                    ROS_INFO("14");
                    curr_cut = curr_paths[0];
                    ROS_INFO("18");
                    ROS_INFO("current cut:%ld",curr_cut.poses.size());
                    ROS_INFO("current paths:%ld",curr_paths.size());
                    sign.data = cal_direction(curr_cut);
                    ROS_INFO("16");
                    epose = curr_cut.poses[0].pose;//路径起始位置
                    fpose = curr_cut.poses[curr_cut.poses.size()-1].pose;//目标位置
                    eposep.header.frame_id="map";
                    eposep.pose=epose;
                    fposep.header.frame_id="map";
                    fposep.pose=fpose;
                    ROS_INFO("17");
                    eposepub_.publish(eposep);
                    fposepub_.publish(fposep);
                    signpub_.publish(sign);
                    pathpub.publish(curr_cut);
                    ROS_INFO("15");
                }
                else
                {
                    ROS_INFO("19");
                    curr_path = msg;
                    curr_paths = pathcut(curr_path);
                    ROS_INFO("20");
                    curr_cut = curr_paths[0];
                    ROS_INFO("20.5");
                    ROS_INFO("current_len:%ld",curr_cut.poses.size());
                    sign.data = cal_direction(curr_cut);
                    ROS_INFO("21");
                    epose = curr_cut.poses[0].pose;//路径起始位置
                    fpose = curr_cut.poses[curr_cut.poses.size()-1].pose;//目标位置
                    ROS_INFO("22");
                    eposep.header.frame_id="map";
                    eposep.pose=epose;
                    fposep.header.frame_id="map";
                    fposep.pose=fpose;

                    eposepub_.publish(eposep);
                    fposepub_.publish(fposep);
                    signpub_.publish(sign);
                    pathpub.publish(curr_cut);

                }
                // epose = msg.poses[0].pose;//路径起始位置
                // fpose = msg.poses[pointNum].pose;//目标位置
                // eposep.header.frame_id="map";
                // eposep.pose=epose;
                // fposep.header.frame_id="map";
                // fposep.pose=fpose;
                // eposepub_.publish(eposep);
                // fposepub_.publish(fposep);
                
                // curr_path = msg;
                // pathpub.publish(curr_path);
                ROS_INFO("PUBLISH SUCC!");
            }
            else
            {
                sign.data = cal_direction(curr_cut);

                eposepub_.publish(eposep);
                fposepub_.publish(fposep);
                ROS_INFO("DON'T CHANGE!");
                signpub_.publish(sign);
                pathpub.publish(curr_cut);
            }
        }
        
        
    }
    catch(const std::exception& e)
    {
        ROS_INFO("error 1 %s",e.what());
    }
    
    
    
}
bool obstacal_check(const geometry_msgs::PoseStamped pos,const nav_msgs::OccupancyGrid map)
{
    double mapx = map.info.origin.position.x;
    double mapy = map.info.origin.position.y;
    double resolution = map.info.resolution;
    int cx = int((pos.pose.position.x - mapx)/resolution);
    int cy = int((pos.pose.position.y - mapy)/resolution);
    if(cx<map.info.width && cy<map.info.height && cx>= 0 && cy >= 0)
    {
        if(map.data[cx*map.info.width + cy] > 10)
        {
            return 1;
        }
    }
    return 0;
}
void map_callback(const nav_msgs::OccupancyGrid &msg)
{
    
    if(!curr_cut.poses.empty())
    {
        for(auto pos : curr_cut.poses)
        {
            if(obstacal_check(pos,msg))
            {
                flag = 0;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"path_buf");
    ros::NodeHandle n;
    eposepub_ = n.advertise<geometry_msgs::PoseStamped>("epose",10);
    fposepub_ = n.advertise<geometry_msgs::PoseStamped>("fpose",10);
    pathpub   = n.advertise<nav_msgs::Path>("stable_local",20);
    signpub_  = n.advertise<std_msgs::Int32>("sign",20);
    ros::Subscriber splinePath = n.subscribe("/move_base/TebLocalPlannerROS/local_plan", 20, pathCallback);
    
    ros::Subscriber odomMsgs = n.subscribe("/odom",20,odomCallback);
    ros::Subscriber local_costmap = n.subscribe("/move_base/local_costmap/costmap",20,map_callback);
    ros::spin();
    return 0;
}
