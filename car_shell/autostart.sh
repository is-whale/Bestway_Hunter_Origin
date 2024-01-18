#!/bin/bash
logfile="/root/catkin_ws/running.log"


# 同步环境ros环境变量,启动自动驾驶节点程序
cd /root/catkin_ws
source devel/setup.bash
roslaunch hunter_base hunter_base.launch
echo "车辆自动驾驶程序启动完成..." >> ${logfile}
		

## 运行车辆路线下发与取消下发程序
echo "开始加载车辆路线下发程序..." >> ${logfile}
/usr/bin/python3 /root/catkin_ws/autodriver.py
