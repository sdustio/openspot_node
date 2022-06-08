#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "openspot/imu.h"
#include "openspot/leg.h"
#include "sdquadx/robot.h"
#include "sdquadx/types.h"
#include "tf2_ros/transform_broadcaster.h"

namespace openspot {
class Drive : public rclcpp::Node {
 public:
  Drive(int argc, char *argv[]);

 private:
  bool Init();
  rcl_interfaces::msg::SetParametersResult HandleParametersChanged(std::vector<rclcpp::Parameter> const &params);
  bool HandleDriveTwist(geometry_msgs::msg::Twist::ConstSharedPtr const &msg);
  bool HandleDrivePose(geometry_msgs::msg::Pose::ConstSharedPtr const &msg);
  bool PublishOdometry();

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr drive_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_twist_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  std::shared_ptr<LegImpl> leg_itf_;
  std::shared_ptr<ImuImpl> imu_itf_;
  rclcpp::TimerBase::SharedPtr leg_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;

  rclcpp::TimerBase::SharedPtr ctrl_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_ptr_;

  sdquadx::Options::SharedPtr opts_;
  sdquadx::RobotCtrl::Ptr robot_;
  sdquadx::drive::DriveCtrl::SharedPtr drive_ctrl_;

  sdquadx::drive::Twist drive_twist_;
  sdquadx::drive::Pose drive_pose_;
};

}  // namespace openspot
