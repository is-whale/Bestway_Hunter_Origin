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
        rospy.init_node("hunter_core", anonymous=False)
        #self.wps_buf = []
  
        rospy.Subscriber("/odom", Odometry, self.wpsCallback, queue_size=10)
        rospy.spin()            
            
    def wpsCallback(self, msgs):
        p1 = np.zeros(3)
        np.set_printoptions(suppress=True)  # 取消科学记数法
        np.set_printoptions(threshold=np.inf)  # 数据完整打印
        p1[0] = msgs.pose.pose.position.x
        p1[1] = msgs.pose.pose.position.y
        p1[2] = msgs.pose.pose.orientation.w
        
        print(">>>>" + str(p1))

        # 按照指定形式输出至保存文件中
        i = 0
        for p in p1:
            if i < len(p1)-1:
                ss = ' '
            else:
                ss = ''
            f1.write(str(p) + ss)
            i+=1

        f1.write("\n")
        print("save way point success!!!")
        #print(np.array(self.wps_buf))
        #print("p:"+str(p1))



if __name__ == '__main__':
    filename = './path.txt'
    f1 = open(filename,'w')
    #f1.write("position.x position.y orientation.w" + "\n")
    hunter_core = Hunter_core()
    f1.close()
