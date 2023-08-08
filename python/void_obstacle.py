#!/usr/bin/python3
# -*- coding: utf-8 -*-
# RS-Helios-5515
import sys,os
import time,datetime
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Obstacle():
    def __init__(self):
        self.lidar_safe_distance = 1.5                          # 安全距离为1.5m
        self.lidar_detect_distance = 0.8                        # 避障距离为1.0m
        self.scan_error = 0.2                                   # 过滤掉0.2m内的点云数据
        self.scanlist = [self.lidar_safe_distance] * 3          # 定义距离判断存储列表
        sub = rospy.Subscriber('/scan',LaserScan,self.getScan,queue_size=10)
        rospy.spin()

    def getScan(self,msg):
        #print(msg.ranges)   # 激光扫描角度范围为(-π,π)rad，即(-180°,180°)
        self.angle_min = -3.14159274101
        self.angle_max = 3.14159274101
        self.angle_increment = 0.0174532923847
        self.scan_filter =[]
        self.void_min = 150
        self.void_max = 210
        self.length = len(np.arange(self.angle_min,self.angle_max,self.angle_increment))-1
        #print(self.length)
        for i in range(0,self.length):
            try:
                if self.void_min < i < self.void_max:       #取指定范围内的点进行避障
                    if msg.ranges[i] >= self.scan_error:        #过滤掉无效点
                        self.scan_filter.append(msg.ranges[i])
            except Exception as e:
                continue

        #print(min(self.scan_filter))
        self.update_status()

    def getfollowcmd(self,msgs):
        try:
            self.cmd_lx = msgs.linear.x
            self.cmd_ly = msgs.linear.y
            self.cmd_lz = msgs.linear.z
            self.cmd_ax = msgs.angular.x
            self.cmd_ay = msgs.angular.y
            self.cmd_az = msgs.angular.z
            return self.cmd_lx,self.cmd_ly,self.cmd_lz,self.cmd_ax,self.cmd_ay,self.cmd_az
            
        except Exception as e:
            pass

    def update_status(self):
        cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        twist = Twist()
        self.minscan = min(self.scan_filter)
        self.scanlist.pop(0)                                    # 移除历史scan
        self.scanlist.append(self.minscan)                      # 加入实时scan入距离列表

        if self.minscan <= self.lidar_detect_distance:          # 指定范围内存在障碍物，则停车
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            rospy.loginfo("distance of Obstacle {}".format(min(self.scan_filter)))
        
        if min(self.scanlist) > self.lidar_detect_distance:
            follow_sub = rospy.Subscriber('/smart/cmd_vel2',Twist,self.getfollowcmd,queue_size=10)
            try:
                twist.linear.x = self.cmd_lx
                twist.linear.y = self.cmd_ly
                twist.linear.z = self.cmd_lz
                twist.angular.x = self.cmd_ax
                twist.angular.y = self.cmd_ay
                twist.angular.z = self.cmd_az
            except Exception as e:
                pass
       
        cmd_pub.publish(twist)



if __name__ == '__main__':
    rospy.init_node('void_obstacle')
    try:
        Obstacle()

    except Exception as e:
        pass
