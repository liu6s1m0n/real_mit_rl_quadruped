#include <array>
#include <chrono>
#include <cstring>
#include <thread>

#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

#include "dm_imu/bsp_crc.h"
#include "sensor/imu_driver.hpp"

namespace
{
void writeFloat(ImuDriver::Frame & frame, std::size_t offset, float value)
{
  std::memcpy(frame.data() + offset, &value, sizeof(value));
}

ImuDriver::Frame makeFrame()
{
  ImuDriver::Frame frame{};
  constexpr std::array<std::array<float, 3>, 3> values{{
    {{1.0F, 2.0F, 9.8F}},
    {{0.1F, 0.2F, 0.3F}},
    {{4.0F, 5.0F, 6.0F}}}};
  for (std::size_t record = 0; record < values.size(); ++record) {
    const std::size_t offset = record * 19;
    frame[offset] = 0x55;
    frame[offset + 1] = 0xAA;
    frame[offset + 2] = 0x01;
    frame[offset + 3] = static_cast<std::uint8_t>(record + 1);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      writeFloat(frame, offset + 4 + axis * sizeof(float), values[record][axis]);
    }
    const auto crc = Get_CRC16(frame.data() + offset, 16);
    frame[offset + 16] = static_cast<std::uint8_t>(crc);
    frame[offset + 17] = static_cast<std::uint8_t>(crc >> 8);
  }
  return frame;
}

bool writeAll(int fd, const std::uint8_t * data, std::size_t size)
{
  std::size_t written = 0;
  while (written < size) {
    const auto result = ::write(fd, data + written, size - written);
    if (result <= 0) {return false;}
    written += static_cast<std::size_t>(result);
  }
  return true;
}
}  // namespace

TEST(ImuDriverTest, DecodesAndRejectsCrcCorrupted57ByteFrames)
{
  const auto valid = makeFrame();
  DmImuRawSample sample;
  ASSERT_TRUE(ImuDriver::decodeFrame(valid, sample));
  EXPECT_FLOAT_EQ(sample.acceleration[0], 1.0F);
  EXPECT_FLOAT_EQ(sample.acceleration[2], 9.8F);
  EXPECT_FLOAT_EQ(sample.angular_velocity[1], 0.2F);
  EXPECT_FLOAT_EQ(sample.rpy_degrees[2], 6.0F);

  auto corrupted = valid;
  corrupted[10] ^= 0x01;
  EXPECT_FALSE(ImuDriver::decodeFrame(corrupted, sample));
}

TEST(ImuDriverTest, ResynchronizesWhenAFrameStartsInsideBadCandidate)
{
  int master_fd = -1;
  int slave_fd = -1;
  char slave_name[128]{};
  ASSERT_EQ(openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr), 0);
  const std::string device(slave_name);
  ::close(slave_fd);

  ImuDriver driver(device, 921600);
  ASSERT_TRUE(driver.start());
  const auto valid = makeFrame();
  const std::uint8_t prefix = 0x55;
  ASSERT_TRUE(writeAll(master_fd, &prefix, 1));
  ASSERT_TRUE(writeAll(master_fd, valid.data(), valid.size()));

  DmImuRawSample sample;
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
}
