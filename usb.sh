#!/bin/bash
logfile="/root/catkin_ws/usb_can.log"

[[ ! -f ${logfile} ]] && touch ${logfile}

echo "USB转CAN模块开始加载..." >> logfile
#加载USB转CAN驱动并设置传输速率为500K
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000

name=`ifconfig | grep "can0" | awk -F ":" '{print $1}'`
if [ ${name} != '' ];then
	echo "USB转CAN模块加载完成..." >> ${logfile}
else
	echo "USB转CAN模块加载失败..." >> ${logfile}
fi
