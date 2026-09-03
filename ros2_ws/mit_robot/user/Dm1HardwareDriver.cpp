#include "Dm1HardwareDriver.hpp"

#include <cstdio>
#include <utility>

Dm1HardwareDriver::Dm1HardwareDriver(
  std::string can_interface, std::string imu_device)
: can_interface_(std::move(can_interface)), imu_device_(std::move(imu_device))
{
}

bool Dm1HardwareDriver::open()
{
  // HARDWARE INTEGRATION POINT 1:
  // 1. 打开 can_interface_ 对应的 SocketCAN/CAN-FD 或达妙 SDK；
  // 2. 打开 imu_device_；
  // 3. 确认 12 台电机均在线，但不要在这里使能；
  // 4. 全部成功后设置 opened_=true 并返回 true。
  std::fprintf(
    stderr,
    "DM1 hardware SDK is not connected. Implement Dm1HardwareDriver::open() "
    "for CAN '%s' and IMU '%s'.\n",
    can_interface_.c_str(), imu_device_.c_str());
  opened_ = false;
  return false;
}

bool Dm1HardwareDriver::read(
  dm1_hardware::HardwareSample & sample, double now_s)
{
  // HARDWARE INTEGRATION POINT 2 (真实数据读取位置):
  // - 解码 IMU 四元数、机身坐标角速度和包含重力的加速度，写入 sample.imu；
  // - 解码 12 台电机的位置/速度/估算力矩/温度/母线电压/故障码；
  // - 电机反馈保持原始电机坐标，Dm1MitInterface 会按标定转换到模型坐标；
  // - 所有 timestamp 使用 HardwareBridge 传入的 now_s 同一单调时间基准。
  static_cast<void>(sample);
  static_cast<void>(now_s);
  return opened_ && false;
}

bool Dm1HardwareDriver::sendMit(const dm1_hardware::MitFrame & frame)
{
  // HARDWARE INTEGRATION POINT 4:
  // 按达妙协议打包 frame.id/position/velocity/kp/kd/torque 并发送一帧。
  // 任何编码或发送错误必须返回 false，HardwareBridge 会立即 disableAll()。
  static_cast<void>(frame);
  return opened_ && false;
}

bool Dm1HardwareDriver::setMotorZero(std::uint8_t motor_id)
{
  // HARDWARE INTEGRATION POINT 3:
  // 电机保持失能，在机器狗确认处于机械趴卧且静止时，发送达妙
  // “当前位置设为零位”命令。等待设备确认后返回 true。
  static_cast<void>(motor_id);
  return opened_ && false;
}

void Dm1HardwareDriver::disableAll() noexcept
{
  // HARDWARE INTEGRATION POINT 5:
  // 向所有已知电机发送失能帧。此函数必须 noexcept、可重复调用，并且在
  // 部分初始化失败时也安全。
}

void Dm1HardwareDriver::close() noexcept
{
  disableAll();
  // 在此关闭 IMU、CAN 文件描述符或厂商 SDK 句柄。
  opened_ = false;
}
