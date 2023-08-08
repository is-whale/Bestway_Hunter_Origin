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


def callback():
    simplegoal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped, queue_size=10)  # 发布节点名称: followingpath
    rospy.init_node("goalpub", anonymous=True)                   # 初始化节点 名称:pubpath
    rate = rospy.Rate(1)                                                        # 设置ros发布频率，单
    poseList = []
    with open(pathfile,'r') as lines:
        for line in lines:
            rows = line.strip('\n').split('|')
            posestamped = PoseStamped()                                                 # 初始化消息类型
            posestamped.header.frame_id = "map"
            posestamped.header.stamp = rospy.get_rostime()                                     # 获取ros当前时间
            posestamped.pose.position.x = float(rows[0])
            posestamped.pose.position.y = float(rows[1]) 
            posestamped.pose.position.z = float(rows[2])
            posestamped.pose.orientation.x = float(rows[3]) 
            posestamped.pose.orientation.y = float(rows[4])
            posestamped.pose.orientation.z = float(rows[5])
            posestamped.pose.orientation.w = float(rows[6])
            poseList.append(posestamped)
    
    while not rospy.is_shutdown():
        simplegoal_pub.publish(poseList[0])
        #poseList.pop(0)
        rate.sleep()


if __name__ == '__main__':
    pathfile = "/root/catkin_ws/python/goal.txt"
    try:
        callback()
    except rospy.ROSInterruptException:
        pass
                                




