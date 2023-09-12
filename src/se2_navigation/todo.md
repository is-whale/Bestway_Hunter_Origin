rostopic info /prius/front_left_laser/scan
Type: sensor_msgs/LaserScan

Publishers:

* /gazebo (http://whale-ROG-G14:44347/)

Subscribers:

* /rviz_car_demo (http://whale-ROG-G14:42527/)

 whale@whale-ROG-G14  ~/code/ros/algorithm/PP  rostopic info /prius/base_pose_ground_truth
Type: nav_msgs/Odometry

Publishers:

* /gazebo (http://whale-ROG-G14:44347/)

Subscribers:

* /prius_controller_node (http://whale-ROG-G14:34647/)
* /fake_localization (http://whale-ROG-G14:39125/)

 whale@whale-ROG-G14  ~/code/ros/algorithm/PP  rostopic info /joint_states
Type: sensor_msgs/JointState

Publishers:

* /gazebo (http://whale-ROG-G14:44347/)

Subscribers:

* /robot_state_publisher (http://whale-ROG-G14:44375/)

 
 whale@whale-ROG-G14  ~/code/ros/algorithm/PP  rostopic echo /prius_control
WARNING: topic [/prius_control] does not appear to be published yet
^C%                                                                                                                                                   whale@whale-ROG-G14  ~/code/ros/algorithm/PP  rostopic info /prius_controls
Type: prius_msgs/Control

Publishers:

* /prius_controller_node (http://whale-ROG-G14:46733/)

Subscribers:

* /gazebo (http://whale-ROG-G14:36207/)

 whale@whale-ROG-G14  ~/code/ros/algorithm/PP  rostopic info /prius_controls
Type: prius_msgs/Control

Publishers:

* /prius_controller_node (http://whale-ROG-G14:46733/)

Subscribers:

* /gazebo (http://whale-ROG-G14:36207/)

 whale@whale-ROG-G14  ~/code/ros/algorithm/PP  rosmsg show prius_msgs/Control
uint8 NO_COMMAND=0
uint8 NEUTRAL=1
uint8 FORWARD=2
uint8 REVERSE=3
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float64 throttle
float64 brake
float64 steer
uint8 shift_gears
