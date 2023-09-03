#!/usr/bin/python3.6
# -*- coding: utf-8 -*-
import sys,os
import time,datetime
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def cmd_vel_convert(msgs):
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    try:
        lx = msgs.linear.x
        ly = msgs.linear.y
        lz = msgs.linear.z
        ax = msgs.angular.x
        ay = msgs.angular.y
        az = msgs.angular.z
        #print(lx,ly,lz)
        if lx > 0:
            lx_pub = 0.5
        elif lx < 0:
            lx_pub = -0.5
        
        twist.linear.x = lx_pub
        twist.linear.y = ly
        twist.linear.z = lz
        twist.angular.x = ax
        twist.angular.y = ay
        twist.angular.z = az
    except Exception as e:
        #print(e)
        pass

    cmd_pub.publish(twist)



if __name__ == '__main__':
    rospy.init_node('teb_buffer')
    rospy.Subscriber('/buffer/cmd_vel',Twist, cmd_vel_convert,queue_size=10)
    rospy.spin()
