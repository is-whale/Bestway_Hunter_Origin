cmake_minimum_required(VERSION 3.0.2)
project(sonic_localization)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL EIGEN3
)

include_directories(
	include
    src
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

## catkin_install_python(PROGRAMS script/receiver.py
## 	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##
## )




# add_executable(get_map src/get_map.cpp)
# add_dependencies(get_map ${catkin_EXPORTED_TARGETS} )
# target_link_libraries(get_map ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(localization src/localization.cpp src/lidarOptimization.cpp)
target_link_libraries(localization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})
