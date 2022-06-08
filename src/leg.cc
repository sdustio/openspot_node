#include "openspot/leg.h"

#include <array>

#include "openspot/spi.h"

namespace openspot {

using sdquadx::consts::model::kNumJoint;

namespace leg {
constexpr char const kSPId1[] = "/dev/spidev2.0";
constexpr char const kSPId2[] = "/dev/spidev2.1";
std::array<std::string const, kNumJoint> const kJointNames = {
    "fr_abad_joint", "fr_hip_joint", "fr_knee_joint", "fl_abad_joint", "fl_hip_joint", "fl_knee_joint",
    "hr_abad_joint", "hr_hip_joint", "hr_knee_joint", "hl_abad_joint", "hl_hip_joint", "hl_knee_joint"};
}  // namespace leg

LegImpl::LegImpl(JointPubPtr pub) : joint_pub_(pub) { InitSpi(leg::kSPId1, leg::kSPId2); }

bool LegImpl::ReadTo(sdquadx::sensor::LegDatas &data) const { return ReadOutTo(data); }

bool LegImpl::WriteFrom(sdquadx::interface::LegCmds const &cmds) { return WriteInFrom(cmds); }

bool LegImpl::RunOnce(rclcpp::Time const &now) {
  RunSpi();
  sdquadx::sensor::LegDatas jtdata;
  ReadOutTo(jtdata);

  sensor_msgs::msg::JointState jtmsg;
  jtmsg.header.stamp = now;
  jtmsg.name.resize(kNumJoint);
  jtmsg.position.resize(kNumJoint);
  jtmsg.velocity.resize(kNumJoint);

  for (std::size_t idx = 0; idx < kNumJoint; ++idx) {
    auto leg = idx / 3;
    auto j = idx % 3;
    auto position = jtdata[leg].q[j];
    auto velocity = jtdata[leg].qd[j];
    jtmsg.name[idx] = leg::kJointNames[idx];
    jtmsg.position[idx] = position;
    jtmsg.velocity[idx] = velocity;
  }

  return true;
}
}  // namespace openspot
