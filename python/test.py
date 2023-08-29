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


class Hunter_core():
    def __init__(self):
        rospy.init_node("hunter_core", anonymous=True)
        #self.wps_buf = []
  
        rospy.Subscriber("/odom", Odometry, self.wpsCallback, queue_size=10)
        rospy.spin()            
            
    def wpsCallback(self, msgs):
        p1 = np.zeros(7)
        np.set_printoptions(suppress=True)  # 取消科学记数法
        np.set_printoptions(threshold=np.inf)  # 数据完整打印
        p1[0] = msgs.pose.pose.position.x
        p1[1] = msgs.pose.pose.position.y
        p1[2] = msgs.pose.pose.position.z
        p1[3] = msgs.pose.pose.orientation.x
        p1[4] = msgs.pose.pose.orientation.y
        p1[5] = msgs.pose.pose.orientation.z
        p1[6] = msgs.pose.pose.orientation.w
        
        print(">>>>" + str(p1))
        for p in p1:
            f1.write(str(p) + '|')
        f1.write("\n")
        print("save way point success!!!")
        #print(np.array(self.wps_buf))
        #print("p:"+str(p1))



if __name__ == '__main__':
    filename = './wps.txt'
    f1 = open(filename,'w')
    f1.write("position.x|position.y|position.z|orientation.x|orientation.y|orientation.z|orientation.w" + "\n")
    hunter_core = Hunter_core()
    f1.close()
