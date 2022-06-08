#pragma once

#include <cinttypes>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "spotng/interface.h"
#include "sensor_msgs/msg/imu.hpp"
#include "serial/serial.h"

// quat_enu = Q_sensor2robot * ( quat * Q_enu2ref)

namespace openspot {

constexpr inline std::size_t const kImuSerialBufferSize = 4096;
constexpr inline std::size_t const kImuDataBufferSize = 1024;

using ImuPubPtr = rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr;

class ImuImpl : public spotng::interface::Imu {
 public:
  ImuImpl(ImuPubPtr pub);
  ~ImuImpl();
  bool ReadTo(spotng::sensor::ImuData &data) const override;
  bool RunOnce(rclcpp::Time const &now);

 private:
  bool ParseData(std::size_t size, rclcpp::Time const &now);
  bool ResetDataParse();

  ImuPubPtr imu_pub_;
  sensor_msgs::msg::Imu imu_;

  serial::Serial serial_;  //串口实例

  std::uint8_t buffer_[kImuSerialBufferSize];

  // yesense imu
  enum class Mode : std::uint8_t {
    HEADER1,
    HEADER2,
    TID_L,
    TID_H,
    LENGTH,
    MESSAGE,
    CHECKSUM_L,
    CHECKSUM_H,
    GPS_RAW = 10
  };
  Mode mode_ = Mode::HEADER1;
  std::size_t index_ = 0;
  std::size_t bytes_ = 0;
  std::uint16_t tid_ = 0x00;
  std::uint16_t prev_tid_ = 0x00;
  std::uint8_t ck1_;
  std::uint8_t ck2_;

  std::uint8_t message_in_[kImuDataBufferSize];
};
}  // namespace openspot
