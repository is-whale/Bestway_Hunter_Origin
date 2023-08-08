## 编译
+ catkin_make  -DCATKIN_WHITELIST_PACKAGES="sonic_mapping" -j8
+ catkin_make  -DCATKIN_WHITELIST_PACKAGES="sonic_localization" -j8

## 建图
+ roslaunch sonic_mapping mapping.launch bag_filename:=**/root/map_zidong/lslibrary.bag** ws_path:=**/root/catkin_ws/src/sonic_slam**

+ roslaunch sonic_mapping mapping.launch
 ws_path:= /root/catkin_ws/src/sonic_slam

+ roslaunch sonic_mapping mapping.launch bag_filename:=**/media/yjsx/Elements/dataset/gaoxin/library.bag** ws_path:=**/home/yjsx/catkin_yjsx/src/sonic_slam**


+ **粗体**为需要修改的位置，第一个写rosbag所在路径，第二个写到sonic_slam的路径
+ mapping.launch里可修改订阅激光点云topic的名字


## 构建costmap
+ python  tool/costmap_generation.py **/home/yjsx/catkin_yjsx/src/sonic_slam**

+ **粗体**为sonic_slam的路径
+ 生成的地图为data/map/costmap.png，初始生成的效果不是很好，可以与用ps把噪点擦掉，把障碍物补全

## 定位
+ roslaunch sonic_localization localization.launch bag_filename:=**/media/yjsx/Elements/dataset/gaoxin/library.bag** ws_path:=**/home/yjsx/catkin_yjsx/src/sonic_slam**


+ **粗体**为需要修改的位置，第一个写rosbag所在路径，第二个写到sonic_slam的路径


附带了一个rosbag在data目录下，可以进行算法验证。
