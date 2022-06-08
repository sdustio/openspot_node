#include "openspot/drive.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<openspot::Drive>(argc, argv));
  rclcpp::shutdown();

  return 0;
}
