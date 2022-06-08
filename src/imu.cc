#include "openspot/imu.h"

#include <chrono>
#include <cmath>
#include <thread>

#include "openspot/consts.h"
#include "openspot/yesense.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace openspot {

namespace imu {
constexpr char const kImuPort[] = "/dev/ttyUSB0";
constexpr std::uint32_t kBaudrate = 460800;
constexpr char const kImuFrame[] = "imu_link";
}  // namespace imu

namespace {
inline double deg2rad(double d) { return d * kPI / 180.; }
}  // namespace

ImuImpl::ImuImpl(ImuPubPtr pub) : imu_pub_(pub) {
  serial_.setPort(imu::kImuPort);
  serial_.setBaudrate(imu::kBaudrate);
  while (!serial_.isOpen()) {
    try {
      serial_.open();
    } catch (serial::IOException &e) {
      printf("Unable to open serial port: %s ,Trying again in 1 second.\n", serial_.getPort().c_str());
      std::this_thread::sleep_for(std::chrono::seconds(1));
      (void)e;
    }
  }
  imu_.header.frame_id = imu::kImuFrame;
}

ImuImpl::~ImuImpl() {
  if (serial_.isOpen()) {
    serial_.close();
  }
}

bool ImuImpl::ReadTo(sdquadx::sensor::ImuData &data) const {
  data.quat = {imu_.orientation.w, imu_.orientation.x, imu_.orientation.y, imu_.orientation.z};
  data.gyro = {imu_.angular_velocity.x, imu_.angular_velocity.y, imu_.angular_velocity.z};
  data.acc = {imu_.linear_acceleration.x, imu_.linear_acceleration.y, imu_.linear_acceleration.z};
  return true;
}

bool ImuImpl::RunOnce(rclcpp::Time const &now) {
  auto sz = std::min(kImuSerialBufferSize, serial_.available());
  if (sz > 0) {
    return ParseData(serial_.read(buffer_, sz), now);
  }
  return true;
}

// IMU Data protocal
/*--------------------------------------------------------------------------------------------------------------
 * 输出协议为：header1(0x59) + header2(0x53) + tid(2B) + payload_len(1B) + payload_data(Nbytes) + ck1(1B) + ck2(1B)
 * crc校验从TID开始到payload data的最后一个字节
 */
bool ImuImpl::ParseData(std::size_t size, rclcpp::Time const &now) {
  for (size_t i = 0; i < size; i++) {
    std::uint8_t data = buffer_[i];

    if (mode_ == Mode::MESSAGE) {
      /* message data being recieved */
      ck1_ += data;
      ck2_ += ck1_;

      // assert index_ < kImuDataBufferSize;
      message_in_[index_++] = data;
      bytes_--;

      if (bytes_ == 0) mode_ = Mode::CHECKSUM_L;
    } else if (mode_ == Mode::HEADER1) {
      if (data == 0x59) mode_ = Mode::HEADER2;
    } else if (mode_ == Mode::HEADER2) {
      if (data == 0x53) {
        // New frame begin
        ck1_ = 0;
        ck2_ = 0;
        index_ = 0;
        mode_ = Mode::TID_L;
      } else {
        mode_ = Mode::HEADER1;
      }
    } else if (mode_ == Mode::TID_L) {
      ck1_ += data;
      ck2_ += ck1_;
      // set tid_ low
      tid_ = data;
      mode_ = Mode::TID_H;
    } else if (mode_ == Mode::TID_H) {
      ck1_ += data;
      ck2_ += ck1_;

      tid_ |= ((uint16_t)data) << 8;

      if (prev_tid_ != 0 && tid_ > prev_tid_ && prev_tid_ != tid_ - 1) {
        printf("Frame losed: prev_TID: %d, cur_TID: %d\n", prev_tid_, tid_);
      }

      prev_tid_ = tid_;

      mode_ = Mode::LENGTH;

    } else if (mode_ == Mode::LENGTH) {
      ck1_ += data;
      ck2_ += ck1_;
      bytes_ = data;
      if (bytes_ == 0) {
        ResetDataParse();
        continue;
      }
      mode_ = Mode::MESSAGE;

    } else if (mode_ == Mode::CHECKSUM_L) {
      if (ck1_ == data) {
        mode_ = Mode::CHECKSUM_H;
      } else {
        ResetDataParse();
        continue;
      }
    } else if (mode_ == Mode::CHECKSUM_H) {
      if (ck2_ == data) {
        // 解析数据
        std::size_t pos = 0;
        std::size_t payload_len = index_;
        payload_data_t *payload = nullptr;
        unsigned char ret = 0xff;

        while (payload_len > 0) {
          payload = (payload_data_t *)(message_in_ + pos);
          // assert pos < kImuDataBufferSize;
          ret =
              parse_data_by_id(payload->data_id, payload->data_len, (unsigned char *)payload + sizeof(payload_data_t));
          if (static_cast<std::uint8_t>(ret) == 0x01) {
            pos += payload->data_len + sizeof(payload_data_t);
            payload_len -= payload->data_len + sizeof(payload_data_t);
          } else {
            // check failed
            pos++;
            payload_len--;
          }
        }

        tf2::Quaternion qt;
        qt.setRPY(deg2rad(g_output_info.roll), deg2rad(g_output_info.pitch), deg2rad(g_output_info.yaw));
        imu_.orientation = tf2::toMsg(qt);

        imu_.angular_velocity.x = deg2rad(g_output_info.angle_x);
        imu_.angular_velocity.y = deg2rad(g_output_info.angle_y);
        imu_.angular_velocity.z = deg2rad(g_output_info.angle_z);

        imu_.linear_acceleration.x = g_output_info.accel_x;
        imu_.linear_acceleration.y = g_output_info.accel_y;
        imu_.linear_acceleration.z = g_output_info.accel_z;

        imu_.header.stamp = now;

        imu_pub_->publish(imu_);
      }
      ResetDataParse();
    }
  }

  return true;
}

bool ImuImpl::ResetDataParse() {
  ck1_ = 0;
  ck2_ = 0;
  index_ = 0;
  mode_ = Mode::HEADER1;
  bytes_ = 0;
  return true;
}

}  // namespace openspot
