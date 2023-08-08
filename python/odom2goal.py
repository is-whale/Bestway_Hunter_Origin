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


def callback(msgs):
    #nowtime = rospy.get_rostime()          # 获取ros当前时间
    posestamped = PoseStamped()             # 初始化消息类型
    #posestamped.header.seq  = msgs.header.seq
    #posestamped.header.stamp.secs = msgs.header.stamp.secs
    #posestamped.header.stamp.nsecs = msgs.header.stamp.nsecs
    #posestamped.header.stamp.secs = nowtime.secs
    #posestamped.header.stamp.nsecs = nowtime.nsecs
    #posestamped.header.frame_id = msgs.header.frame_id
    posestamped.header.frame_id = "map"
    posestamped.pose.position.x = msgs.pose.pose.position.x 
    posestamped.pose.position.y = msgs.pose.pose.position.y 
    posestamped.pose.position.z = msgs.pose.pose.position.z 
    posestamped.pose.orientation.x = msgs.pose.pose.orientation.x 
    posestamped.pose.orientation.y = msgs.pose.pose.orientation.y 
    posestamped.pose.orientation.z = msgs.pose.pose.orientation.z 
    posestamped.pose.orientation.w = msgs.pose.pose.orientation.w 
    simplegoal_pub.publish(posestamped)


if __name__ == '__main__':
    simplegoal_pub = rospy.Publisher('move_base_simple/goal',PoseStamped, queue_size=10)                                            # 定义发布节点名称
    rospy.init_node("path_odom", anonymous=True)         # 初始化节点 名称:odom  
    rospy.Subscriber("path_odom",Odometry,callback)        # 订阅节点
    rospy.spin()                                             # 阻塞线程
