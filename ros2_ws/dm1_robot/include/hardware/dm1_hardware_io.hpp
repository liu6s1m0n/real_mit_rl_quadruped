/*! @file dm1_hardware_io.hpp
 *  @brief HardwareBridge 与具体 DM1 CAN/IMU SDK 之间的唯一适配接口。
 */
#ifndef DM1_ROBOT_HARDWARE_DM1_HARDWARE_IO_HPP_
#define DM1_ROBOT_HARDWARE_DM1_HARDWARE_IO_HPP_

#include "hardware/dm1_mit_interface.hpp"
#include "common/robot_types.hpp"

namespace dm1_hardware
{

/// 一次硬件读取得到的 IMU 和 12 路电机原始反馈。
struct HardwareSample
{
  ImuData<float> imu;                       ///< IMU 采样。
  Dm1MitInterface::FeedbackArray motors{};  ///< 电机坐标系下的原始反馈。
};

/**
 * @brief 真机硬件抽象：在 MitTransport 之上增加打开、读取、置零和使能。
 *
 * 真机由 Dm1HardwareDriver 实现，仿真和测试可替换为其他实现。
 */
class Dm1HardwareIo : public MitTransport
{
public:
  virtual ~Dm1HardwareIo() = default;

  /** 打开 CAN、IMU 和必要的实时设备；失败时不得使能电机。 */
  virtual bool open() = 0;
  /** 一次读取同步的 IMU 与 12 路电机反馈。 */
  virtual bool read(HardwareSample & sample, double now_s) = 0;
  /**
   * 持续读取 IMU，不要求电机再次主动返回反馈。
   * 默认实现兼容旧的测试硬件；真实驱动可直接读取 IMU 串口。
   */
  virtual bool readImu(ImuData<float> & sample, double now_s)
  {
    HardwareSample combined;
    if (!read(combined, now_s)) {return false;}
    sample = combined.imu;
    return true;
  }
  /** 将指定电机的当前位置写为输出轴零位。 */
  virtual bool setMotorZero(const MotorAddress & address) = 0;
  /** 在不使能输出的前提下轮询全部电机反馈。 */
  virtual bool pollMotors() = 0;
  /** 显式使能全部电机输出。 */
  virtual bool enableAll() = 0;
  /** 关闭设备；实现必须先失能全部电机。 */
  virtual void close() noexcept = 0;
};

}  // namespace dm1_hardware

#endif  // DM1_ROBOT_HARDWARE_DM1_HARDWARE_IO_HPP_
