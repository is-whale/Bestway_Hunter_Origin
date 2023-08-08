#!/bin/bash
logfile="/root/catkin_ws/running.log"
lidarIP="192.168.1.202"

[[ ! -f ${logfile} ]] && touch ${logfile}

echo "USB转CAN模块开始加载..." >> ${logfile}
# 加载USB转CAN驱动并设置传输速率为500K
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000

name=`ifconfig | grep "can0" | awk -F ":" '{print $1}'`
if [ ${name} != '' ];then
	echo "USB转CAN模块加载完成..." >> ${logfile}
else
	echo "USB转CAN模块加载失败..." >> ${logfile}
fi

# 开启自动驾驶程序
while true
do
	# 需等待VNC远程桌面成功开启后再启动ros节点
	vino=`netstat -tlnp | grep "vino-server" | head -n 1 | awk '{print $NF}'`
	echo ${vino}
	if [ ${vino} != '' ];then
		# 判断激光雷达是否在线
		code=`ping -w 2 -c 3 ${lidarIP} | grep packet | awk -F" " '{print $6}'| awk -F"%" '{print $1}'`
                echo ${code}
	      	# 启动车辆自动驾驶lanunch文件
		if [ ${code} == 0 ];then

			# 同步环境ros环境变量,启动自动驾驶节点程序
			cd /root/catkin_ws
			source devel/setup.bash
			roslaunch hunter_base hunter_base.launch
			break
		else
			sleep 1
			continue
		fi	
	else
		sleep 2
	fi
done
echo "车辆自动驾驶程序启动完成..." >> ${logfile}
		

# 运行车辆路线下发与取消下发程序
echo "开始加载车辆路线下发程序..." >> ${logfile}
usr/bin/python3 /root/catkin_ws/autodriver.py
