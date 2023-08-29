#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys,os
import time,datetime
import rospy
import numpy as np
import threading
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def callback(msgs):
    #print(msgs.twist.twist.linear)
    cmd_pub = rospy.Publisher('xunji_vel',Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = msgs.twist.twist.linear.x
    twist.linear.y = msgs.twist.twist.linear.y
    twist.linear.z = msgs.twist.twist.linear.z
    twist.angular.x = msgs.twist.twist.angular.x
    twist.angular.y = msgs.twist.twist.angular.y
    twist.angular.z = msgs.twist.twist.angular.z
    cmd_pub.publish(twist)


def publish(self):
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    cmd_pub.publish(twist)
    rospy.loginfo("distance of Obstacle {}".format(self.lidar_detect_distance))


if __name__ == '__main__':
    rospy.init_node("odom", anonymous=True)         # 初始化节点 名称:hunter_odom  
    rospy.Subscriber('odom',Odometry,callback)        # 订阅节点
    rospy.spin()                                             # 阻塞线程
