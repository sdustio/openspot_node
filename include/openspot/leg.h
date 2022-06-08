#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sdquadx/interface.h"
#include "sensor_msgs/msg/joint_state.hpp"

namespace openspot {
using JointPubPtr = rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr;

class LegImpl : public sdquadx::interface::Leg {
 public:
  LegImpl(JointPubPtr pub);
  bool ReadTo(sdquadx::sensor::LegDatas &data) const override;
  bool WriteFrom(sdquadx::interface::LegCmds const &cmds) override;
  bool RunOnce(rclcpp::Time const &now);

 private:
  JointPubPtr joint_pub_;
};
}  // namespace openspot
