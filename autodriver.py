#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys,os
import time,datetime
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
import threading
import socket


aps_connect_ = False
udp_aps_socket = ''
aps_addr = ''
initmode = '待机模式'       # 定义初始车辆模式

def aps_connect():
  global aps_connect_
  global udp_aps_socket
  global aps_addr
  """
  连接驾驶舱服务器
  """
  try:
    # 配置当前目标主机地址
    apsIP = '192.168.1.200'
    # 配置当前目的主机端口
    apsPort = int(9999)        
    # 配置当前本地主机端口
    ldPort = int(9999)   

    print("当前连接的目标主机地址:%s，目的端口:%s，本地端口:%s" %(apsIP,apsPort,ldPort))

   
    #  创建socket套接字
    udp_aps_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    #  绑定端口port
    ld_addr = ('', ldPort)   # 默认本机任何ip
    udp_aps_socket.bind(ld_addr)     # 绑定本地端口
    udp_aps_socket.settimeout(1.5)     # 设置socket超时时间

    aps_addr = (apsIP,apsPort) # 绑定接收方地址、端口
    aps_connect_ = True 

    
  except Exception as e:
    print(e)
    udp_aps_socket.close()
    aps_connect_ = False


def receivemessage():
  global aps_connect_
  global udp_aps_socket         
 
  while True:
    try:
        if (aps_connect_ == True):
            recv_data = udp_aps_socket.recvfrom(1024)
            recvmsg = recv_data[0].hex()
            send_addr = recv_data[1]
            #print(send_addr,recvmsg)
            if recvmsg == '08000004210200000000000000':              # 判断接收到02数据时为自动模式
                carmode = '自动模式'
            rospub(carmode)
        else:
            print("警告", "远程驾驶舱连接中断！") 
            time.sleep(1)
        
    except Exception as e:
        carmode = '待机模式'                # 判断指定超时时间内未收到数据即设为待机模式
        rospub(carmode)
        print(e)
        continue


def rospub(carmode):
  global initmode
  
  # 设置ros预置点位置信息
  goal_msg = PoseStamped()
  goal_msg.header.frame_id = 'map'
  goal_msg.pose.position.x = 18.7336597443
  goal_msg.pose.position.y = -0.514739990234
  goal_msg.pose.position.z = 0.0
  goal_msg.pose.orientation.x = 0.0
  goal_msg.pose.orientation.y = 0.0
  goal_msg.pose.orientation.z = -0.02
  goal_msg.pose.orientation.w = 0.99

  # 设置取消预置点信息
  cancel_msg = GoalID()
  #cancel.publish(cancel_msg)
  #print(carmode)
  
  if carmode != initmode:
    if carmode == '自动模式':       # 自动模式下下发预置点
        #print(goal_msg)
        pub.publish(goal_msg)
    elif carmode == '待机模式':     # 待机模式下取消预置点
        #print(cancel_msg)
        cancel.publish(cancel_msg)

  initmode = carmode                # 更新初始模式为当前模式，作为后续模式信号变化量比较


if __name__ == '__main__':
  # 连接远程驾驶舱服务器
  aps_connect()
  
  # 初始化发布节点/move_base，topic为/move_base_simple/goal，类型为PoseStamped
  rospy.init_node("move_base", anonymous=True)
  pub = rospy.Publisher('move_base_simple/goal',PoseStamped, queue_size=10)
  cancel = rospy.Publisher('move_base/cancel',GoalID, queue_size=10)

  # 开启数据接收函数
  thread = threading.Thread(target=receivemessage)
  thread.setDaemon(False)  # 守护线程
  thread.start()  # 开启子线程 
