/*! @file imu.hpp
 *  @brief 统一的 IMU 数据源：可选择从仿真器（MuJoCo）或真实硬件读取，
 *         每次读取结果统一保存为 ImuData<float>。
 */

#ifndef MYMIT_ROBOT_SENSOR_IMU_HPP_
#define MYMIT_ROBOT_SENSOR_IMU_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "model/robot_types.hpp"

/**
 * @brief IMU 数据来源类型。
 *
 * SIMULATOR：从 MuJoCo 仿真器读取；HARDWARE：接收真实硬件驱动注入的采样。
 */
enum class ImuSource : std::uint8_t
{
  SIMULATOR = 0,
  HARDWARE = 1
};

/**
 * @brief 硬件 IMU 驱动向控制层提交的直接测量。
 *
 * rpy_world_from_body 使用 ZYX 欧拉角约定，顺序为 roll/pitch/yaw，单位 rad。
 * 角速度和角加速度均在机身坐标系中。部分设备没有线加速度时保持
 * acceleration_valid=false，不能用角加速度代替线加速度。
 */
struct HardwareImuMeasurement
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vec3<float> rpy_world_from_body = Vec3<float>::Zero();
  Vec3<float> angular_velocity_body = Vec3<float>::Zero();
  Vec3<float> angular_acceleration_body = Vec3<float>::Zero();
  Vec3<float> acceleration_body = Vec3<float>::Zero();
  float timestamp = 0.0F;
  bool acceleration_valid = false;
  bool valid = false;
};

class ImuDriver;

/**
 * @brief 统一的 IMU 数据源接口。
 *
 * 仿真器与真实硬件各自实现 read()，返回相同格式的 ImuData<float>。
 * 硬件实现应把设备输出的姿态角转换为 orientation_world_from_body，
 * 而不是只上传原始陀螺仪数据交给 OrientationEstimator 积分；
 * 每次读取的结果都会保存到成员 imu 中，供控制循环直接访问。
 */
class ImuSensor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~ImuSensor() = default;

  /** @brief 读取一次 IMU 数据；结果同时保存到成员 imu 中。 */
  virtual ImuData<float> read() = 0;

  /** @brief 最近一次 read() 读取并保存的 IMU 采样数据。 */
  ImuData<float> imu;
};

/**
 * @brief 仿真器 IMU 数据源：从 MuJoCo 仿真数据读取。
 *
 * 构造时完成传感器名称、类型和维度检查，并缓存 sensordata 地址；
 * read() 在仿真控制循环中直接调用，不会重复查找传感器名称。
 */
class SimImu : public ImuSensor
{
public:
  /**
   * @brief 构造函数：绑定 MuJoCo 模型与仿真数据，并定位三个 IMU 传感器。
   *
   * @param model               MuJoCo 模型指针，不能为空（为空会抛异常）。
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

  ImuData<float> read() override;

private:
  /**
   * @brief 按名称查找传感器并校验其类型、维度与数据地址。
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

  // 绑定的 MuJoCo 仿真数据指针，read() 时从中读取 sensordata
  const mjData * data_ = nullptr;
  // 姿态传感器（四元数）数据在 sensordata 缓冲区中的起始地址
  int orientation_address_ = -1;
  // 角速度传感器（陀螺仪）数据在 sensordata 缓冲区中的起始地址
  int angular_velocity_address_ = -1;
  // 加速度传感器（加速度计）数据在 sensordata 缓冲区中的起始地址
  int acceleration_address_ = -1;
};

/**
 * @brief 真实硬件 IMU 数据源。
 *
 * 该类直接打开并读取 IMU 串口，在 imu.cpp 内完成协议校验、数据转换和姿态
 * 四元数计算。设备端必须已经配置为持续输出数据；本类只接收，不向设备发送
 * 配置命令。无串口参数构造时仍保留 update()/read() 的注入模式，供仿真和单元测试使用。
 */
class HardwareImu : public ImuSensor
{
public:
  HardwareImu() = default;
  explicit HardwareImu(std::string serial_device, int baudrate = 921600);
  ~HardwareImu() override;

  HardwareImu(const HardwareImu &) = delete;
  HardwareImu & operator=(const HardwareImu &) = delete;

  /** @brief 打开并配置本地接收串口；不会向 IMU 发送任何命令。 */
  bool open();

  /** @brief 关闭接收串口。 */
  void close() noexcept;

  /**
   * @brief 从串口接收并转换一帧 DM IMU 数据。
   * @param sample 输出的控制层 IMU 数据。
   * @param timestamp 控制循环使用的单调时间戳，单位秒。
   */
  bool readAt(ImuData<float> & sample, float timestamp);

  /**
   * @brief 注入硬件驱动读取的一帧 IMU 数据。
   * @return 数值有效并已保存时返回 true；无效帧会清空缓存并返回 false。
   */
  bool update(const ImuData<float> & sample);

  /**
   * @brief 注入硬件直接输出的 RPY、角速度和角加速度，不做姿态积分。
   */
  bool update(const HardwareImuMeasurement & measurement);

  ImuData<float> read() override;

private:
  std::string serial_device_;
  int baudrate_ = 921600;
  std::unique_ptr<ImuDriver> driver_;
};

/**
 * @brief 按来源类型创建 IMU 数据源。
 *
 * @param source 数据来源：仿真器或真实硬件。
 * @param model  仿真器模式需要的 MuJoCo 模型指针；硬件模式可传 nullptr。
 * @param data   仿真器模式需要的 MuJoCo 数据指针；硬件模式可传 nullptr。
 * @return 指向 IMU 数据源对象的 unique_ptr；未知来源时抛出 std::invalid_argument。
 */
std::unique_ptr<ImuSensor> makeImu(
  ImuSource source, const mjModel * model = nullptr, const mjData * data = nullptr);

#endif  // MYMIT_ROBOT_SENSOR_IMU_HPP_
