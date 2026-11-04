/*! @file imu.hpp
 *  @brief IMU 数据源：MuJoCo 仿真传感器与 DM IMU 串口，统一输出 ImuData<float>。
 */

#ifndef DM1_ROBOT_SENSOR_IMU_HPP_
#define DM1_ROBOT_SENSOR_IMU_HPP_

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "common/robot_types.hpp"

class ImuDriver;

/**
 * @brief 仿真器 IMU 数据源：从 MuJoCo 仿真数据读取。
 *
 * 构造时完成传感器名称、类型和维度检查，并缓存 sensordata 地址；
 * read() 在仿真控制循环中直接调用，不会重复查找传感器名称。
 */
class SimImu
{
public:
  /**
   * @brief 构造函数：绑定 MuJoCo 模型与仿真数据，并定位三个 IMU 传感器。
   *
   * @param model               MuJoCo 模型指针。
   * @param data                MuJoCo 仿真数据指针，read() 从其中读取 sensordata。
   * @param orientation_sensor   姿态传感器的名称，类型须为 FRAMEQUAT（四元数）。
   *                           默认为 "imu_orientation"。
   * @param angular_velocity_sensor 角速度传感器的名称，类型须为 GYRO（陀螺仪）。
   *                           默认为 "imu_angular_velocity"。
   * @param acceleration_sensor 加速度传感器的名称，类型须为 ACCELEROMETER（加速度计）。
   *                           默认为 "imu_linear_acceleration"。
   */
  explicit SimImu(
    const mjModel * model, const mjData * data,
    const std::string & orientation_sensor = "imu_orientation",
    const std::string & angular_velocity_sensor = "imu_angular_velocity",
    const std::string & acceleration_sensor = "imu_linear_acceleration");

  /** @brief 读取当前仿真时刻的姿态、角速度和加速度。 */
  ImuData<float> read() const;

private:
  /**
   * @brief 按名称查找传感器并校验其类型与维度。
   *
   * @param model             MuJoCo 模型指针。
   * @param name              要查找的传感器名称（如 "imu_orientation"）。
   * @param expected_type     期望的传感器类型（如 mjSENS_FRAMEQUAT）。
   * @param expected_dimension 期望的传感器维度（姿态为 4，角速度/加速度为 3）。
   * @return 该传感器数据在 sensordata 缓冲区中的起始偏移地址。
   */
  static int requireSensor(
    const mjModel * model, const std::string & name,
    int expected_type, int expected_dimension);

  const mjData * data_;                ///< 绑定的 MuJoCo 仿真数据，read() 时从中读取 sensordata。
  int orientation_address_ = -1;       ///< 姿态四元数在 sensordata 中的起始地址。
  int angular_velocity_address_ = -1;  ///< 陀螺仪角速度在 sensordata 中的起始地址。
  int acceleration_address_ = -1;      ///< 加速度计数据在 sensordata 中的起始地址。
};

/**
 * @brief 真实硬件 IMU 数据源。
 *
 * 该类直接打开并读取 IMU 串口，在 imu.cpp 内完成协议校验、数据转换和姿态
 * 四元数计算。设备端必须已经配置为持续输出数据；本类只接收，不向设备发送
 * 配置命令。
 */
class HardwareImu
{
public:
  /// @param serial_device 串口设备路径；@param baudrate 串口波特率。
  explicit HardwareImu(std::string serial_device, int baudrate = 921600);
  /// 析构时关闭串口和接收线程。
  ~HardwareImu();

  HardwareImu(const HardwareImu &) = delete;
  HardwareImu & operator=(const HardwareImu &) = delete;

  /** @brief 打开并配置本地接收串口；不会向 IMU 发送任何命令。 */
  bool open();

  /** @brief 关闭接收串口。 */
  void close() noexcept;

  /**
   * @brief 串口掉线（USB 重新枚举/瞬断）后重新打开并重建启动基准。
   *
   * 会先彻底关掉旧的 fd 和接收线程，再用同一设备路径重新 open()；
   * open() 内含静止标定，因此机器人不在静止状态时重连会失败并稍后重试。
   * @return 重新可用时返回 true。
   */
  bool tryRecover();

  /**
   * @brief 从串口接收并转换一帧 DM IMU 数据。
   * @param sample 输出的控制层 IMU 数据。
   * @param timestamp 控制循环使用的单调时间戳，单位秒。
   */
  bool readAt(ImuData<float> & sample, float timestamp);

private:
  /** @brief 在串口接收线程已有有效帧后采集稳定的启动基准。 */
  bool calibrateStartupBaseline();

  std::string serial_device_;           ///< 串口设备路径。
  int baudrate_ = 921600;               ///< 串口波特率。
  std::unique_ptr<ImuDriver> driver_;   ///< 后台串口接收驱动。
  Vec3<float> startup_gravity_body_ = Vec3<float>::Zero();  ///< 启动静止时机身系下的平均重力，m/s^2。
  Vec3<float> startup_rpy_degrees_ = Vec3<float>::Zero();   ///< 启动静止时的平均 roll/pitch/yaw，度。
  float startup_gravity_magnitude_ = 9.81F;                 ///< 启动静止时测得的重力模长，m/s^2。
  /// 仅消除启动航向角；保留启动 roll/pitch，使原始重力向量与姿态处于同一世界系。
  Eigen::Quaternionf startup_heading_orientation_ = Eigen::Quaternionf::Identity();
  bool startup_baseline_valid_ = false;  ///< 启动基准是否已采集。
  /// IMU 安装偏差：传感器坐标系相对机体坐标系的旋转 R_body_sensor（roll/pitch/yaw，度）。
  /// 串口路径输出前统一转换：R_wb = R_ws * R_bs^T，a_b = R_bs * a_s，w_b = R_bs * w_s。
  Vec3<float> mount_rpy_degrees_ = Vec3<float>::Zero();
  Eigen::Quaternionf mount_body_from_sensor_ = Eigen::Quaternionf::Identity();  ///< 安装偏差四元数。
  /// 加速度计模长归一化系数：9.81 / 启动静止时测得的 |g|。
  /// 下游估计器（LinearKalmanFilter）固定用 9.81 抵消重力，模长偏差会变成持续的虚假加速度。
  float acceleration_scale_ = 1.0F;
};

#endif  // DM1_ROBOT_SENSOR_IMU_HPP_
