/*
 * PriusControllerRos.hpp
 *
 *  Created on: Apr 6, 2020
 *      Author: jelavice
 */

#pragma once
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include "car_demo/PIDController.hpp"
#include "prius_msgs/PriusControl.hpp"
#include "se2_navigation_msgs/Path.hpp"
#include "se2_navigation_msgs/RequestCurrentStateSrv.h"
#include "se2_navigation_msgs/SendControllerCommandSrv.h"

namespace pure_pursuit {
class PathTracker;
class Path;
}  // namespace pure_pursuit

namespace car_demo {

class PriusControllerRos {
  using CurrentStateService = se2_navigation_msgs::RequestCurrentStateSrv;
  using ControllerCommandService = se2_navigation_msgs::SendControllerCommandSrv;

 public:
  PriusControllerRos(ros::NodeHandlePtr nh);
  virtual ~PriusControllerRos(); /* = default(), defined in cpp file*/
  void initialize(double dt);
  void advance();

 private:
  void update();
  void translateCommands(double longitudinalSpeed, double steeringAngle, prius_msgs::PriusControl* ctrl);
  void translateGear(double longitudinalSpeed, prius_msgs::PriusControl* ctrl) const;
  void translateVelocity(double desiredVelocityMagnitude, prius_msgs::PriusControl* ctrl);
  void createPathTrackerAndLoadParameters();
  void loadPIDParameters();
  void publishControl(const prius_msgs::PriusControl& ctrl) const;
  void initRos();
  void priusStateCallback(const nav_msgs::Odometry& odometry);
  void pathCallback(const se2_navigation_msgs::PathMsg& pathMsg);
  void stopTracking();
  bool currentStateRequestService(CurrentStateService::Request& req, CurrentStateService::Response& res);
  bool controllerCommandService(ControllerCommandService::Request& req, ControllerCommandService::Response& res);

  void processStartTrackingCommand();
  void processAbortTrackingCommand();
  //增加cmdvel发布函数
  void PriusControllerRos::publishCmdvel(const const CurrentStateService::Response& cmd_vel) const;

  ros::NodeHandlePtr nh_;
  double dt_ = 0.01;
  ros::Publisher priusControlPub_;
  ros::Subscriber priusStateSub_;
  ros::Subscriber pathSub_;
  ros::ServiceServer priusCurrentStateService_;
  ros::ServiceServer controllerCommandService_;
  nav_msgs::Odometry priusState_;

  //start：cmd_vel数据发送
  ros::Publisher cmdVelPub_;
  //end

  /*state machine variables*/
  bool planReceived_ = false;
  bool currentlyExecutingPlan_ = false;
  bool receivedStartTrackingCommand_ = false;
  bool doneFollowingPrev_ = false;
  bool publishTrackingStatus_ = false;

  prius_msgs::PriusControl priusControl_;
  std::unique_ptr<pure_pursuit::PathTracker> pathTracker_;
  PIDController pidController_;
  se2_navigation_msgs::Path currentPath_;
};

double longitudinalVelocity(const nav_msgs::Odometry& odom);
void convert(se2_navigation_msgs::Path& path, pure_pursuit::Path* out);
} /* namespace car_demo*/
