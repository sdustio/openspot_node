#pragma once

#include <cstdint>

#include "sdquadx/interface.h"

/*!
 * SPI command message
 */
struct SpiCmd {
  float q_des_abad[2];
  float q_des_hip[2];
  float q_des_knee[2];
  float qd_des_abad[2];
  float qd_des_hip[2];
  float qd_des_knee[2];
  float kp_abad[2];
  float kp_hip[2];
  float kp_knee[2];
  float kd_abad[2];
  float kd_hip[2];
  float kd_knee[2];
  float tau_abad_ff[2];
  float tau_hip_ff[2];
  float tau_knee_ff[2];
  uint32_t flags[2];
  uint32_t checksum;
};

/*!
 * SPI data message
 */
struct SpiData {
  float q_abad[2];
  float q_hip[2];
  float q_knee[2];
  float qd_abad[2];
  float qd_hip[2];
  float qd_knee[2];
  uint32_t flags[2];
  uint32_t checksum;
};

bool InitSpi(char const *spi1, char const *spi2);

bool RunSpi();

bool ReadOutTo(sdquadx::sensor::LegDatas &data);

bool WriteInFrom(sdquadx::interface::LegCmds const &cmds);
