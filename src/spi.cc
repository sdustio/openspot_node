#include "openspot/spi.h"

#ifdef __cplusplus
extern "C" {
#endif
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <unistd.h>
#ifdef __cplusplus
}
#endif

#include <cstdio>
#include <cstring>

#include "openspot/consts.h"

namespace consts {
constexpr std::uint32_t const kSpiLen = sizeof(SpiCmd);
constexpr std::uint8_t const kSpiMode = SPI_MODE_0;
constexpr std::uint8_t const kSpiBitsPerWord = 8;
constexpr std::uint32_t const kSpiSpeed = 6000000;
constexpr std::uint8_t const kSpiLsb = 0x01;
constexpr std::size_t const kSpiCmdLenOfUint16 = sizeof(SpiCmd) / sizeof(std::uint16_t);
constexpr std::size_t const kSpiDataLenOfUint16 = sizeof(SpiData) / sizeof(std::uint16_t);
constexpr std::size_t const kSpiCmdChecksumBytes = sizeof(SpiCmd) - sizeof(std::uint32_t);
constexpr std::size_t const kSpiDataChecksumBytes = sizeof(SpiData) - sizeof(std::uint32_t);

constexpr float const kKneeOffsetPos = 4.35f;

// only used for actual robot
constexpr float const kAbadSideSign[4] = {-1.f, -1.f, 1.f, 1.f};
constexpr float const kHipSideSign[4] = {-1.f, 1.f, -1.f, 1.f};
constexpr float const kKneeSideSign[4] = {-.6429f, .6429f, -.6429f, .6429f};

// only used for actual robot
constexpr float const kAbadOffset[4] = {0.05f, -0.05f, -0.07f, 0.07f};
constexpr float const kHipOffset[4] = {kPI / 2.f, -kPI / 2.f, -kPI / 2.f, kPI / 2.f};
constexpr float const kKneeOffset[4] = {kKneeOffsetPos, -kKneeOffsetPos, kKneeOffsetPos, -kKneeOffsetPos};
}  // namespace consts

namespace gd {
sdquadx::interface::LegCmds leg_cmds = {};
sdquadx::sensor::LegDatas leg_datas = {};

SpiCmd spi_cmd;
SpiData spi_data;

pthread_mutex_t spi_mutex;

int spi_1_fd = -1;
int spi_2_fd = -1;

// transmit and receive buffers
std::uint16_t tx_buf[consts::kSpiCmdLenOfUint16];
std::uint16_t rx_buf[consts::kSpiCmdLenOfUint16];
}  // namespace gd

namespace {
std::uint32_t xor_checksum(std::uint32_t *data, std::size_t len) {
  std::uint32_t t = 0;
  for (std::size_t i = 0; i < len; i++) t = t ^ data[i];
  return t;
}
}  // namespace

void DataFromSpi(std::size_t);
void CmdToSpi(std::size_t);

bool InitSpi(char const *spi1, char const *spi2) {
  std::memset(&gd::spi_cmd, 0, sizeof(SpiCmd));
  std::memset(&gd::spi_data, 0, sizeof(SpiData));

  if (pthread_mutex_init(&gd::spi_mutex, nullptr) != 0) {
    std::perror("[ERROR: RT SPI] Failed to create spi data mutex\n");
    return false;
  }
  gd::spi_1_fd = open(spi1, O_RDWR);
  if (gd::spi_1_fd < 0) {
    std::perror("[ERROR] Couldn't open spidev 2.0\n");
    return false;
  }
  gd::spi_2_fd = open(spi2, O_RDWR);
  if (gd::spi_2_fd < 0) {
    std::perror("[ERROR] Couldn't open spidev 2.1\n");
    return false;
  }

  int rv = 0;

  std::uint8_t m = consts::kSpiMode;
  rv = ioctl(gd::spi_1_fd, SPI_IOC_WR_MODE, &m);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_WR_MODE (1)\n");
    return false;
  }
  rv = ioctl(gd::spi_2_fd, SPI_IOC_WR_MODE, &m);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_WR_MODE (2)\n");
    return false;
  }
  rv = ioctl(gd::spi_1_fd, SPI_IOC_RD_MODE, &m);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_RD_MODE (1)\n");
    return false;
  }
  rv = ioctl(gd::spi_2_fd, SPI_IOC_RD_MODE, &m);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_RD_MODE (2)\n");
    return false;
  }

  std::uint8_t w = consts::kSpiBitsPerWord;
  rv = ioctl(gd::spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &w);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_WR_BITS_PER_WORD (1)\n");
    return false;
  }
  rv = ioctl(gd::spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &w);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_WR_BITS_PER_WORD (2)\n");
    return false;
  }
  rv = ioctl(gd::spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &w);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_RD_BITS_PER_WORD (1)\n");
    return false;
  }
  rv = ioctl(gd::spi_2_fd, SPI_IOC_RD_BITS_PER_WORD, &w);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_RD_BITS_PER_WORD (2)\n");
    return false;
  }

  std::uint32_t s = consts::kSpiSpeed;
  rv = ioctl(gd::spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &s);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_WR_MAX_SPEED_HZ (1)\n");
    return false;
  }
  rv = ioctl(gd::spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &s);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_WR_MAX_SPEED_HZ (2)\n");
    return false;
  }
  rv = ioctl(gd::spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &s);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_RD_MAX_SPEED_HZ (1)\n");
    return false;
  }
  rv = ioctl(gd::spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &s);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_RD_MAX_SPEED_HZ (2)\n");
    return false;
  }

  std::uint8_t l = consts::kSpiLsb;
  // rv = ioctl(gd::spi_1_fd, SPI_IOC_WR_LSB_FIRST, &l);
  // if (rv < 0) {
  //   std::perror("[ERROR] ioctl SPI_IOC_WR_LSB_FIRST (1)\n");
  //   return false;
  // }
  // rv = ioctl(gd::spi_2_fd, SPI_IOC_WR_LSB_FIRST, &l);
  // if (rv < 0) {
  //   std::perror("[ERROR] ioctl SPI_IOC_WR_LSB_FIRST (2)\n");
  //   return false;
  // }
  rv = ioctl(gd::spi_1_fd, SPI_IOC_RD_LSB_FIRST, &l);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_RD_LSB_FIRST (1)\n");
    return false;
  }
  rv = ioctl(gd::spi_2_fd, SPI_IOC_RD_LSB_FIRST, &l);
  if (rv < 0) {
    std::perror("[ERROR] ioctl SPI_IOC_RD_LSB_FIRST (2)\n");
    return false;
  }

  return rv == 0;
}

bool RunSpi() {
  pthread_mutex_lock(&gd::spi_mutex);

  int rv = 0;

  for (std::size_t spi_board = 0; spi_board < 2u; spi_board++) {
    CmdToSpi(spi_board * 2);

    // pointers to command/data spine array
    std::uint16_t *cmd_d = (std::uint16_t *)&gd::spi_cmd;
    std::uint16_t *data_d = (std::uint16_t *)&gd::spi_data;

    // zero rx buffer
    std::memset(gd::rx_buf, 0, consts::kSpiLen);

    // copy into tx buffer flipping bytes
    for (std::size_t i = 0; i < consts::kSpiCmdLenOfUint16; i++) {
      gd::tx_buf[i] = (cmd_d[i] >> 8) + ((cmd_d[i] & 0xff) << 8);
    }

    // spi message struct
    struct spi_ioc_transfer spi_message;
    // zero message struct.
    std::memset(&spi_message, 0, sizeof(struct spi_ioc_transfer));
    // set up message struct
    spi_message.bits_per_word = consts::kSpiBitsPerWord;
    spi_message.delay_usecs = 0;
    spi_message.len = consts::kSpiLen;
    spi_message.rx_buf = (uint64_t)gd::rx_buf;
    spi_message.tx_buf = (uint64_t)gd::tx_buf;

    // do spi communication
    rv = ioctl(spi_board == 0 ? gd::spi_1_fd : gd::spi_2_fd, SPI_IOC_MESSAGE(1), &spi_message);
    if (rv < 0) {
      std::perror("SPI Communication Error");
      return false;
    }

    for (std::size_t i = 0; i < consts::kSpiDataLenOfUint16; i++) {
      data_d[i] = (gd::rx_buf[i] >> 8) + ((gd::rx_buf[i] & 0xff) << 8);
    }
    DataFromSpi(spi_board * 2);
  }

  pthread_mutex_unlock(&gd::spi_mutex);
  return true;
}

bool ReadOutTo(sdquadx::sensor::LegDatas &data) {
  data = gd::leg_datas;
  return true;
}

bool WriteInFrom(sdquadx::interface::LegCmds const &cmds) {
  gd::leg_cmds = cmds;
  return true;
}

void DataFromSpi(std::size_t leg_0) {
  for (std::size_t i = 0; i < 2u; i++) {
    gd::leg_datas[i + leg_0].q[0] =
        (gd::spi_data.q_abad[i] - consts::kAbadOffset[i + leg_0]) * consts::kAbadSideSign[i + leg_0];
    gd::leg_datas[i + leg_0].q[1] =
        (gd::spi_data.q_hip[i] - consts::kHipOffset[i + leg_0]) * consts::kHipSideSign[i + leg_0];
    gd::leg_datas[i + leg_0].q[2] =
        (gd::spi_data.q_knee[i] - consts::kKneeOffset[i + leg_0]) * consts::kKneeSideSign[i + leg_0];

    gd::leg_datas[i + leg_0].qd[0] = gd::spi_data.qd_abad[i] * consts::kAbadSideSign[i + leg_0];
    gd::leg_datas[i + leg_0].qd[1] = gd::spi_data.qd_hip[i] * consts::kHipSideSign[i + leg_0];
    gd::leg_datas[i + leg_0].qd[2] = gd::spi_data.qd_knee[i] * consts::kKneeSideSign[i + leg_0];
  }

  std::uint32_t calc_checksum = xor_checksum((std::uint32_t *)&gd::spi_data, consts::kSpiDataChecksumBytes / sizeof(std::uint32_t));
  if (calc_checksum != gd::spi_data.checksum)
    printf("SPI ERROR BAD CHECKSUM GOT 0x%hx EXPECTED 0x%hx\n", calc_checksum, gd::spi_data.checksum);
}

void CmdToSpi(std::size_t leg_0) {
  for (std::size_t i = 0; i < 2u; i++) {
    gd::spi_cmd.q_des_abad[i] =
        (gd::leg_cmds[i + leg_0].q_des[0] * consts::kAbadSideSign[i + leg_0]) + consts::kAbadOffset[i + leg_0];
    gd::spi_cmd.q_des_hip[i] =
        (gd::leg_cmds[i + leg_0].q_des[1] * consts::kHipSideSign[i + leg_0]) + consts::kHipOffset[i + leg_0];
    gd::spi_cmd.q_des_knee[i] =
        (gd::leg_cmds[i + leg_0].q_des[2] / consts::kKneeSideSign[i + leg_0]) + consts::kKneeOffset[i + leg_0];

    gd::spi_cmd.qd_des_abad[i] = gd::leg_cmds[i + leg_0].qd_des[0] * consts::kAbadSideSign[i + leg_0];
    gd::spi_cmd.qd_des_hip[i] = gd::leg_cmds[i + leg_0].qd_des[1] * consts::kHipSideSign[i + leg_0];
    gd::spi_cmd.qd_des_knee[i] = gd::leg_cmds[i + leg_0].qd_des[2] / consts::kKneeSideSign[i + leg_0];

    gd::spi_cmd.kp_abad[i] = gd::leg_cmds[i + leg_0].kp[0];
    gd::spi_cmd.kp_hip[i] = gd::leg_cmds[i + leg_0].kp[1];
    gd::spi_cmd.kp_knee[i] = gd::leg_cmds[i + leg_0].kp[2];

    gd::spi_cmd.kd_abad[i] = gd::leg_cmds[i + leg_0].kd[0];
    gd::spi_cmd.kd_hip[i] = gd::leg_cmds[i + leg_0].kd[1];
    gd::spi_cmd.kd_knee[i] = gd::leg_cmds[i + leg_0].kd[2];

    gd::spi_cmd.tau_abad_ff[i] = gd::leg_cmds[i + leg_0].tau[0] * consts::kAbadSideSign[i + leg_0];
    gd::spi_cmd.tau_hip_ff[i] = gd::leg_cmds[i + leg_0].tau[1] * consts::kHipSideSign[i + leg_0];
    gd::spi_cmd.tau_knee_ff[i] = gd::leg_cmds[i + leg_0].tau[2] * consts::kKneeSideSign[i + leg_0];

    gd::spi_cmd.flags[i] = 1;
  }
  gd::spi_cmd.checksum = xor_checksum((std::uint32_t *)&gd::spi_cmd, consts::kSpiCmdChecksumBytes / sizeof(std::uint32_t));
}
