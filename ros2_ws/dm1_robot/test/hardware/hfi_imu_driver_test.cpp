/*! @file hfi_imu_driver_test.cpp
 *  @brief HfiImuDriver 帧解码、符号换算和串口重同步测试。
 */
#include <array>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

#include "sensor/hfi_imu_driver.hpp"

namespace
{
std::uint16_t crc16Modbus(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001U) :
        static_cast<std::uint16_t>(crc >> 1);
    }
  }
  return crc;
}

/// 按厂商格式组帧：AA 55 LEN [3..6] [7..] 浮点 CRC_lo CRC_hi；floats[0] 为时间戳位。
std::vector<std::uint8_t> makeFrame(std::uint8_t length, const std::vector<float> & floats)
{
  std::vector<std::uint8_t> frame(static_cast<std::size_t>(length) + 5U, 0);
  frame[0] = 0xAA;
  frame[1] = 0x55;
  frame[2] = length;
  for (std::size_t i = 0; i < floats.size(); ++i) {
    std::memcpy(frame.data() + 7 + i * sizeof(float), &floats[i], sizeof(float));
  }
  const auto crc = crc16Modbus(frame.data() + 2, static_cast<std::size_t>(length) + 1U);
  frame[length + 3] = static_cast<std::uint8_t>(crc);
  frame[length + 4] = static_cast<std::uint8_t>(crc >> 8);
  return frame;
}

std::vector<std::uint8_t> makeInertialFrame()
{
  // 时间戳、角速度 rad/s、加速度（重力矢量，g；水平静止 z=-1）、磁场
  return makeFrame(
    HfiImuDriver::kInertialLength,
    {0.0F, 0.1F, -0.2F, 0.3F, 0.01F, -0.02F, -1.0F, 30.0F, 40.0F, 50.0F});
}

std::vector<std::uint8_t> makeEulerFrame()
{
  return makeFrame(HfiImuDriver::kEulerLength, {0.0F, 5.0F, -10.0F, 90.0F});
}

bool writeAll(int fd, const std::vector<std::uint8_t> & data)
{
  std::size_t written = 0;
  while (written < data.size()) {
    const auto result = ::write(fd, data.data() + written, data.size() - written);
    if (result <= 0) {return false;}
    written += static_cast<std::size_t>(result);
  }
  return true;
}
}  // namespace

// 惯性帧：重力矢量 g → 比力 m/s^2（取反），角速度原样；CRC 损坏时拒绝。
TEST(HfiImuDriverTest, DecodesInertialFrameAndRejectsCorruptedCrc)
{
  const auto frame = makeInertialFrame();
  ASSERT_EQ(frame.size(), HfiImuDriver::kInertialFrameSize);
  std::array<float, 3> gyro{};
  std::array<float, 3> acc{};
  ASSERT_TRUE(HfiImuDriver::decodeInertialFrame(frame.data(), frame.size(), gyro, acc));
  EXPECT_FLOAT_EQ(gyro[0], 0.1F);
  EXPECT_FLOAT_EQ(gyro[1], -0.2F);
  EXPECT_FLOAT_EQ(gyro[2], 0.3F);
  EXPECT_NEAR(acc[0], -0.01F * 9.80665F, 1e-5F);
  EXPECT_NEAR(acc[2], 9.80665F, 1e-5F);

  auto corrupted = frame;
  corrupted[20] ^= 0x01;
  EXPECT_FALSE(
    HfiImuDriver::decodeInertialFrame(corrupted.data(), corrupted.size(), gyro, acc));
  EXPECT_FALSE(HfiImuDriver::decodeInertialFrame(frame.data(), frame.size() - 1, gyro, acc));
}

// 欧拉角帧：按厂商 ROS 驱动约定取反 pitch/yaw，转换到右手系。
TEST(HfiImuDriverTest, DecodesEulerFrameIntoRightHandedConvention)
{
  const auto frame = makeEulerFrame();
  ASSERT_EQ(frame.size(), HfiImuDriver::kEulerFrameSize);
  std::array<float, 3> rpy{};
  ASSERT_TRUE(HfiImuDriver::decodeEulerFrame(frame.data(), frame.size(), rpy));
  EXPECT_FLOAT_EQ(rpy[0], 5.0F);
  EXPECT_FLOAT_EQ(rpy[1], 10.0F);
  EXPECT_FLOAT_EQ(rpy[2], -90.0F);
}

// 串口流中夹杂垃圾字节和截断的伪帧头时，能重新对齐并合成一个样本。
TEST(HfiImuDriverTest, ResynchronizesAndPairsInertialWithEulerFrame)
{
  int master_fd = -1;
  int slave_fd = -1;
  char slave_name[128]{};
  ASSERT_EQ(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr), 0);
  const std::string device(slave_name);
  ::close(slave_fd);

  HfiImuDriver driver(device, 921600);
  ASSERT_TRUE(driver.start());
  ASSERT_TRUE(writeAll(master_fd, {0x12, 0xAA, 0x55, 0x2C, 0x00, 0xAA, 0x34}));
  const auto inertial = makeInertialFrame();
  const auto euler = makeEulerFrame();
  ASSERT_TRUE(writeAll(master_fd, inertial));
  // 只有惯性帧时不应发布样本。
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ImuRawSample sample;
  EXPECT_FALSE(driver.latest(sample, std::chrono::milliseconds(200)));
  ASSERT_TRUE(writeAll(master_fd, euler));

  bool received = false;
  for (int attempt = 0; attempt < 100 && !received; ++attempt) {
    received = driver.latest(sample, std::chrono::milliseconds(200));
    if (!received) {std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  }
  driver.stop();
  ::close(master_fd);

  ASSERT_TRUE(received);
  EXPECT_EQ(sample.sequence, 1U);
  EXPECT_FLOAT_EQ(sample.angular_velocity[2], 0.3F);
  EXPECT_NEAR(sample.acceleration[2], 9.80665F, 1e-5F);
  EXPECT_FLOAT_EQ(sample.rpy_degrees[2], -90.0F);
}
