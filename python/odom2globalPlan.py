#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys,os
import time,datetime
import rospy
import numpy as np
import threading
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path


def odom2globalPlan(msgs):
    posestamped = PoseStamped()               # 初始化消息类型,PoseStamped
    posestamped.header.stamp = rospy.get_rostime()
    posestamped.header.frame_id = "map"
    posestamped.pose.position.x = msgs.pose.pose.position.x
    posestamped.pose.position.y = msgs.pose.pose.position.y
    posestamped.pose.position.z = 0.0
    posestamped.pose.orientation.w = 1.0
    path.poses.append(posestamped)
    simplegoal_pub.publish(path)


if __name__ == '__main__':
    simplegoal_pub = rospy.Publisher('/followingpath',Path, queue_size=10)                                            # 定义发布节点名称
    rospy.init_node("pubpath", anonymous=True)         # 初始化节点 名称:pubpath  
    path = Path()                                   # 初始化消息类型,Path
    path.header.frame_id = "map"
    nowtime = rospy.get_rostime()                   # 获取ros当前时间
    path.header.stamp = nowtime
    rospy.Subscriber("path_odom",Odometry,odom2globalPlan)        # 订阅节点
    rospy.spin()                                             # 阻塞线程                        
