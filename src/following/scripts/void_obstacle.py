#!/usr/bin/python3.6
# -*- coding: utf-8 -*-
import sys,os
import time,datetime
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class Obstacle():
    def __init__(self):
        self.lidar_detect_distance = 0.7   #避障距离为0.8m
        self.scan_front_filter = 0.3    #过滤掉0.2m内的前向点云数据
        self.scan_edge_filter = 0.5     #过滤掉0.5m内的侧向点云数据

        sub = rospy.Subscriber('/scan',LaserScan,self.getScan,queue_size=10)
        rospy.spin()

    def getScan(self,msg):
        #print(msg.ranges)   # 激光扫描角度范围为(-π,π)rad，即(-180°,180°)
        self.angle_min = -3.14159274101
        self.angle_max = 3.14159274101
        self.angle_increment = 0.0174532923847
        #self.scan_filter =[]
        scan_dict = {}          # 缓存障碍物角度与距离关系字典
        self.scan_sorted = {}   # 按距离排序后的关系字典
        length = len(np.arange(self.angle_min,self.angle_max,self.angle_increment))-1
        #print(length)
        for i in range(0,length):
            try:
                angle = i - 180     # 换算成对应角度值
                if -40 <= angle <= 40:   # 取前向激光扫描角度
                    if msg.ranges[i] >= self.scan_front_filter:
                        scan_dict[angle] = msg.ranges[i]

                else:                      # 取侧向激光扫描角度
                    if msg.ranges[i] >= self.scan_edge_filter:
                        scan_dict[angle] = msg.ranges[i]

            except Exception as e:
                continue

        self.scan_sorted = dict(sorted(scan_dict.items(),key=lambda x: x[1]))
        angle = list(self.scan_sorted.keys())[0]
        distance = self.scan_sorted[angle]
        #print("障碍物角度%s,距离%s" %(angle,distance))
        self.update_status(angle,distance)

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

    def update_status(self,angle,distance):
        cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        twist = Twist()
        
        if distance <= self.lidar_detect_distance:
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            cmd_pub.publish(twist)
            #rospy.loginfo("障碍物角度值{}°,距离值{}m".format(angle,distance))
        else:
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

    rospy.init_node('stop_car')
    try:
        Obstacle()

    except Exception as e:
        pass

