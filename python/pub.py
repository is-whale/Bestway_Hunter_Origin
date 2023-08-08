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


def track(msgs):
    #nowtime = rospy.get_rostime()          # 获取ros当前时间
    #path = Path()                            # 初始化消息类型
    #path.header.seq  = msgs.header.seq
    #path.header.stamp.secs = msgs.header.stamp.secs
    #path.header.stamp.nsecs = msgs.header.stamp.nsecs
    #path.header.stamp.secs = nowtime.secs
    #path.header.stamp.nsecs = nowtime.nsecs
    #path.header.frame_id = msgs.header.frame_id
    #path.header.frame_id = "map"
    #posestamped.header.stamp = rospy.get_rostime() 
    i = 0
    lennum = len(msgs_list)-1
    for msg in msgs_list:
        if i == 0 or i == lennum:
            posestamped = PoseStamped()               # 初始化消息类型,PoseStamped
            posestamped.header.frame_id = "map"
            posestamped.pose.position.x = float(msg[0]) 
            posestamped.pose.position.y = float(msg[1]) 
            posestamped.pose.position.z = float(msg[2])
            posestamped.pose.orientation.x = float(msg[3])
            posestamped.pose.orientation.y = float(msg[4])
            posestamped.pose.orientation.z = float(msg[5])
            posestamped.pose.orientation.w = float(msg[6])
            path.poses.append(posestamped)
        i+=1
    
def callback(msgs):
    print(path)
    simplegoal_pub.publish(path)


if __name__ == '__main__':
    filename = './wps.txt'
    rospy.init_node("tt", anonymous=True)         # 初始化节点 名称:odom  
    simplegoal_pub = rospy.Publisher('/move_base/xunji_plan',Path, queue_size=10)                                            # 定义发布节点名称
    path = Path()
    path.header.frame_id = "map"
    nowtime = rospy.get_rostime()                   # 获取ros当前时间
    path.header.stamp = nowtime                     
    msgs_list = []
    with open(filename,'r') as lines:
        next(lines)
        for line in lines:
            rows = line.strip("\n").split('|')
            msgs_list.append(rows)
    track(msgs_list)               
    rospy.Subscriber("odom",Odometry,callback)        # 订阅节点
    rospy.spin()                                             # 阻塞线程
