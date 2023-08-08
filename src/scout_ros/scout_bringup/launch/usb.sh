#!/bin/bash
sudo modprobe gs_usb
sudo ip link set can0 up type can bitrate 500000
