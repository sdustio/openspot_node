#include "openspot/drive.h"

#include <cmath>
#include <cstring>

namespace openspot {

namespace ros {
constexpr char const kNodeName[] = "openspot_drive";
constexpr char const kDefaultCmdTwistTopic[] = "/cmd_vel";
constexpr char const kDefaultCmdPoseTopic[] = "/cmd_pose";
}  // namespace ros

namespace drive {
constexpr std::chrono::milliseconds kCtrlDt(static_cast<int>(1.0 / 0.5));    // 0.5kHz
constexpr std::chrono::microseconds kImuDt(static_cast<int>(1000. / 100.));  // 100kHz
constexpr std::chrono::milliseconds kLegDt(static_cast<int>(1.0 / 0.5));     // 0.5kHz

}  // namespace drive

namespace log {
std::unordered_map<std::string, spotng::logging::Level> const kLogLevelMap = {
    {"debug", spotng::logging::Level::Debug},
    {"info", spotng::logging::Level::Info},
    {"warn", spotng::logging::Level::Warn},
    {"err", spotng::logging::Level::Err},
    {"critical", spotng::logging::Level::Critical}};
std::unordered_map<std::string, spotng::logging::Target> const kLogTargetMap = {
    {"console", spotng::logging::Target::Console},
    {"file", spotng::logging::Target::File},
    {"rotate_file", spotng::logging::Target::RotateFile}};
}  // namespace log

namespace odom {
constexpr char const kOdometryFrame[] = "odom";
constexpr char const kRobotBaseFrame[] = "base_footprint";
constexpr spotng::fpt_t const kCovariance[3] = {0.01, 0.01, 0.01};
}  // namespace odom

namespace {
template <typename T>
T Square(T a) {
  return a * a;
}
}  // namespace

Drive::Drive(int argc, char *argv[]) : Node(ros::kNodeName), opts_(std::make_shared<spotng::Options>()) {
  opts_->ctrl_sec = drive::kCtrlDt.count() / 1'000.0;
  // Parse arguments
  for (int i = 1; i < argc; i++) {
    if (std::strcmp(argv[i], "--log_level") == 0) {
      i++;
      opts_->log_level = log::kLogLevelMap.at(argv[i]);
      continue;
    }
    if (std::strcmp(argv[i], "--log_target") == 0) {
      i++;
      opts_->log_target = log::kLogTargetMap.at(argv[i]);
      continue;
    }
    if (std::strcmp(argv[i], "--log_filename") == 0) {
      i++;
      std::strncpy(opts_->log_filename, argv[i], sizeof(opts_->log_filename) - 1);
      continue;
    }
  }

  this->declare_parameter<int>("mode", 0);
  this->declare_parameter<int>("state", 0);
  this->declare_parameter<int>("gait", 0);
  this->declare_parameter<double>("step_height", 0.1);

  Init();
}

bool Drive::Init() {
  imu_itf_ = std::make_shared<ImuImpl>(this->create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS()));
  imu_timer_ = this->create_wall_timer(drive::kImuDt, [this]() { this->imu_itf_->RunOnce(this->now()); });

  leg_itf_ = std::make_shared<LegImpl>(
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_stats", rclcpp::SensorDataQoS()));
  leg_timer_ = this->create_wall_timer(drive::kLegDt, [this]() { this->leg_itf_->RunOnce(this->now()); });

  spotng::RobotCtrl::Build(robot_, opts_, leg_itf_, imu_itf_);

  // sub drive twist and register drive twist handler
  drive_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      ros::kDefaultCmdTwistTopic, rclcpp::ServicesQoS(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr const msg) { this->HandleDriveTwist(msg); });

  // sub drive varpose and reigster drive varpose handler
  drive_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
      ros::kDefaultCmdPoseTopic, rclcpp::ServicesQoS(),
      [this](geometry_msgs::msg::Pose::ConstSharedPtr const msg) { this->HandleDrivePose(msg); });

  // run main ctrl periodically
  ctrl_timer_ = this->create_wall_timer(drive::kCtrlDt, [this]() { this->robot_->RunOnce(); });

  param_callback_ptr_ = this->add_on_set_parameters_callback(
      [this](std::vector<rclcpp::Parameter> const &params) { return this->HandleParametersChanged(params); });

  return true;
}

rcl_interfaces::msg::SetParametersResult Drive::HandleParametersChanged(std::vector<rclcpp::Parameter> const &params) {
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &param : params) {
    auto name = param.get_name();
    if (name == "mode" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      drive_ctrl_->UpdateMode(static_cast<spotng::drive::Mode>(param.as_int()));
    else if (name == "state" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      drive_ctrl_->UpdateState(static_cast<spotng::drive::State>(param.as_int()));
    else if (name == "gait" && param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      drive_ctrl_->UpdateGait(static_cast<spotng::drive::Gait>(param.as_int()));
    else if (name == "step_height" && param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      drive_ctrl_->UpdateStepHeight(param.as_double());
  }
  result.successful = true;
  return result;
}

bool Drive::HandleDriveTwist(geometry_msgs::msg::Twist::ConstSharedPtr const &msg) {
  drive_twist_.lvel_x = msg->linear.x;
  drive_twist_.lvel_y = msg->linear.y;
  /* currently not supported
  drive_twist_.lvel_z = msg->linear.z;
  drive_twist_.avel_x = msg->angular.x;
  drive_twist_.avel_y = msg->angular.y;
  */
  drive_twist_.avel_z = msg->angular.z;

  return drive_ctrl_->UpdateTwist(drive_twist_);
}

bool Drive::HandleDrivePose(geometry_msgs::msg::Pose::ConstSharedPtr const &msg) {
  drive_pose_.height = msg->position.z;

  // q: [w, x, y, z]
  spotng::SdVector4f q = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
  double as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);

  drive_pose_.yaw =
      std::atan2(2 * (q[1] * q[2] + q[0] * q[3]), Square(q[0]) + Square(q[1]) - Square(q[2]) - Square(q[3]));
  drive_pose_.pitch = std::asin(as);
  drive_pose_.roll =
      std::atan2(2 * (q[2] * q[3] + q[0] * q[1]), Square(q[0]) - Square(q[1]) - Square(q[2]) + Square(q[3]));

  return drive_ctrl_->UpdatePose(drive_pose_);
}

bool Drive::PublishOdometry() {
  auto const &est = robot_->GetEstimatState();
  auto const &now = this->now();
  nav_msgs::msg::Odometry odom;

  odom.pose.pose.position.x = est.pos[0];
  odom.pose.pose.position.y = est.pos[1];
  odom.pose.pose.position.z = est.pos[2];

  odom.pose.pose.orientation.w = est.ori[0];
  odom.pose.pose.orientation.x = est.ori[1];
  odom.pose.pose.orientation.y = est.ori[2];
  odom.pose.pose.orientation.z = est.ori[3];

  odom.twist.twist.angular.z = est.avel_robot[2];
  odom.twist.twist.linear.x = est.lvel_robot[0];
  odom.twist.twist.linear.y = est.lvel_robot[1];

  odom.pose.covariance[0] = odom::kCovariance[0];
  odom.pose.covariance[7] = odom::kCovariance[1];
  odom.pose.covariance[14] = 1000000000000.0;
  odom.pose.covariance[21] = 1000000000000.0;
  odom.pose.covariance[28] = 1000000000000.0;
  odom.pose.covariance[35] = odom::kCovariance[2];

  odom.twist.covariance[0] = odom::kCovariance[0];
  odom.twist.covariance[7] = odom::kCovariance[1];
  odom.twist.covariance[14] = 1000000000000.0;
  odom.twist.covariance[21] = 1000000000000.0;
  odom.twist.covariance[28] = 1000000000000.0;
  odom.twist.covariance[35] = odom::kCovariance[2];

  odom.header.stamp = now;
  odom.header.frame_id = odom::kOdometryFrame;
  odom.child_frame_id = odom::kRobotBaseFrame;

  odometry_pub_->publish(odom);

  geometry_msgs::msg::TransformStamped tfs;
  tfs.header.stamp = now;
  tfs.header.frame_id = odom::kOdometryFrame;
  tfs.child_frame_id = odom::kRobotBaseFrame;
  tfs.transform.translation.x = odom.pose.pose.position.x;
  tfs.transform.translation.y = odom.pose.pose.position.y;
  tfs.transform.translation.z = odom.pose.pose.position.z;
  tfs.transform.rotation = odom.pose.pose.orientation;

  transform_broadcaster_->sendTransform(tfs);

  return true;
}

}  // namespace openspot
