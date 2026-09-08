// ============================================================
// IMU 数据源实现。
// 真实硬件路径在本文件内直接完成串口接收、DM 协议校验和数据转换；
// 不再依赖独立的 ROS IMU 接收节点。
// ============================================================

#include "sensor/imu.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

#include "sensor/imu_driver.hpp"
#include "sensor/imu_log.hpp"

// ---------- 内部辅助函数（匿名命名空间） ----------
namespace
{

// 校验传感器数据地址是否合法：
// 地址非负、维度为正，且地址与维度之和不超过传感器数据缓冲区的总长度。
bool sensorRangeIsValid(const mjModel * model, int address, int dimension)
{
  return address >= 0 && dimension > 0 &&
         address <= model->nsensordata - dimension;
}

Eigen::Quaternionf rpyDegreesToQuaternion(const Vec3<float> & rpy_degrees)
{
  constexpr float kDegreesToRadians = 0.01745329251994329577F;
  const float roll = rpy_degrees.x() * kDegreesToRadians;
  const float pitch = rpy_degrees.y() * kDegreesToRadians;
  const float yaw = rpy_degrees.z() * kDegreesToRadians;
  Eigen::Quaternionf result =
    Eigen::AngleAxisf(yaw, Vec3<float>::UnitZ()) *
    Eigen::AngleAxisf(pitch, Vec3<float>::UnitY()) *
    Eigen::AngleAxisf(roll, Vec3<float>::UnitX());
  result.normalize();
  return result;
}

}  // namespace

// ---------- 仿真器 IMU 数据源（SimImu） ----------

// 构造函数：绑定模型与仿真数据，并解析定位三个 IMU 传感器。
SimImu::SimImu(
  const mjModel * model, const mjData * data,
  const std::string & orientation_sensor,
  const std::string & angular_velocity_sensor,
  const std::string & acceleration_sensor)
: data_(data)
{
  // 校验模型指针非空
  if (model == nullptr) {
    throw std::invalid_argument("MuJoCo model must not be null");
  }

  // 定位姿态传感器：FRAMEQUAT（四元数），维度为 4
  orientation_address_ = requireSensor(
    model, orientation_sensor, mjSENS_FRAMEQUAT, 4);
  // 定位角速度传感器：GYRO（陀螺仪），维度为 3
  angular_velocity_address_ = requireSensor(
    model, angular_velocity_sensor, mjSENS_GYRO, 3);
  // 定位加速度传感器：ACCELEROMETER（加速度计），维度为 3
  acceleration_address_ = requireSensor(
    model, acceleration_sensor, mjSENS_ACCELEROMETER, 3);
}

// 读取一次 IMU 数据：从 MuJoCo 仿真数据中提取姿态、角速度、加速度和时间戳，
// 并对读取结果做有限性（finite）与模长校验，校验失败时返回无效数据。
// 每次读取的结果都会同步保存到成员 imu 中。
ImuData<float> SimImu::read()
{
  ImuData<float> result;

  // 数据有效性检查：数据指针为空或时间戳非有限值则直接返回无效数据
  if (data_ == nullptr || data_->sensordata == nullptr ||
    !std::isfinite(static_cast<float>(data_->time)))
  {
    imu = result;  // 保存无效数据，保持成员与返回值一致
    return result;
  }

  // 根据记录的地址偏移，指向各传感器数据在缓冲区中的位置
  const mjtNum * orientation = data_->sensordata + orientation_address_;
  const mjtNum * angular_velocity = data_->sensordata + angular_velocity_address_;
  const mjtNum * acceleration = data_->sensordata + acceleration_address_;

  // 提取原始数据并填充到结果结构体
  // 姿态：世界系到机体系的四元数 (w, x, y, z)
  result.orientation_world_from_body = Eigen::Quaternionf(
    orientation[0], orientation[1], orientation[2], orientation[3]);
  // 角速度：机体坐标系下的 (x, y, z) 分量
  result.angular_velocity_body <<
    angular_velocity[0], angular_velocity[1], angular_velocity[2];
  // 加速度：机体坐标系下的 (x, y, z) 分量
  result.acceleration_body <<
    acceleration[0], acceleration[1], acceleration[2];
  result.gravity_magnitude = 9.81F;
  // 时间戳：当前仿真时间
  result.timestamp = static_cast<float>(data_->time);

  // 校验读取结果：所有数值必须有限，且四元数模长不能过小（防止除零/退化）
  const float quaternion_norm = result.orientation_world_from_body.norm();
  if (!result.orientation_world_from_body.coeffs().allFinite() ||
    !result.angular_velocity_body.allFinite() ||
    !result.acceleration_body.allFinite() ||
    quaternion_norm <= std::numeric_limits<float>::epsilon())
  {
    result = ImuData<float>{};  // 校验失败，重置为无效数据
    imu = result;               // 保存到成员，保持与返回值一致
    return result;
  }

  // 归一化四元数以保证旋转的有效性，并标记数据有效
  result.orientation_world_from_body.normalize();
  result.orientation_valid = true;
  result.acceleration_valid = true;
  result.angular_acceleration_valid = false;
  result.gravity_valid = true;
  result.valid = true;

  // 将本次读取的数据保存到成员 imu，供外部直接访问
  imu = result;
  return result;
}

// 传感器查找与校验：按名称在 MuJoCo 模型中查找传感器，
// 校验其类型、维度和数据地址，全部通过后返回 sensordata 缓冲区中的起始地址。
int SimImu::requireSensor(
  const mjModel * model, const std::string & name,
  int expected_type, int expected_dimension)
{
  // 通过名称获取传感器 ID
  const int sensor_id = mj_name2id(model, mjOBJ_SENSOR, name.c_str());
  // 未找到该传感器则抛出异常
  if (sensor_id < 0) {
    throw std::invalid_argument("MuJoCo IMU sensor not found: " + name);
  }

  // 校验传感器的类型与维度是否与期望一致
  if (model->sensor_type[sensor_id] != expected_type ||
    model->sensor_dim[sensor_id] != expected_dimension)
  {
    throw std::invalid_argument(
            "MuJoCo IMU sensor has unexpected type or dimension: " + name);
  }

  // 获取数据起始地址并校验其范围是否合法
  const int address = model->sensor_adr[sensor_id];
  if (!sensorRangeIsValid(model, address, expected_dimension)) {
    throw std::invalid_argument("MuJoCo IMU sensor has an invalid data address: " + name);
  }
  return address;
}

// ---------- 真实硬件 IMU 数据源 ----------

bool HardwareImu::update(const ImuData<float> & sample)
{
  ImuData<float> validated = sample;
  const bool base_valid = sample.valid &&
    sample.angular_velocity_body.allFinite() &&
    (!sample.acceleration_valid || sample.acceleration_body.allFinite()) &&
    (!sample.angular_acceleration_valid ||
    sample.angular_acceleration_body.allFinite()) &&
    std::isfinite(sample.timestamp);
  if (!base_valid) {
    imu = ImuData<float>{};
    return false;
  }

  if (sample.orientation_valid) {
    const float norm = sample.orientation_world_from_body.norm();
    if (!sample.orientation_world_from_body.coeffs().allFinite() ||
      !std::isfinite(norm) || norm <= std::numeric_limits<float>::epsilon())
    {
      imu = ImuData<float>{};
      return false;
    }
    validated.orientation_world_from_body.normalize();
  } else {
    // 没有磁力计、视觉或设备姿态解算时，不伪造绝对方向。
    validated.orientation_world_from_body = Eigen::Quaternionf::Identity();
  }

  if (!validated.acceleration_valid) {
    validated.acceleration_body.setZero();
  }
  if (!validated.angular_acceleration_valid) {
    validated.angular_acceleration_body.setZero();
  }

  imu = validated;
  return true;
}

bool HardwareImu::update(const HardwareImuMeasurement & measurement)
{
  if (!measurement.valid || !measurement.rpy_world_from_body.allFinite() ||
    !measurement.angular_velocity_body.allFinite() ||
    !measurement.angular_acceleration_body.allFinite() ||
    (measurement.acceleration_valid &&
    !measurement.acceleration_body.allFinite()) ||
    !std::isfinite(measurement.timestamp))
  {
    imu = ImuData<float>{};
    return false;
  }

  const float roll = measurement.rpy_world_from_body.x();
  const float pitch = measurement.rpy_world_from_body.y();
  const float yaw = measurement.rpy_world_from_body.z();
  ImuData<float> sample;
  sample.orientation_world_from_body =
    Eigen::AngleAxisf(yaw, Vec3<float>::UnitZ()) *
    Eigen::AngleAxisf(pitch, Vec3<float>::UnitY()) *
    Eigen::AngleAxisf(roll, Vec3<float>::UnitX());
  sample.angular_velocity_body = measurement.angular_velocity_body;
  sample.angular_acceleration_body = measurement.angular_acceleration_body;
  sample.acceleration_body = measurement.acceleration_body;
  sample.timestamp = measurement.timestamp;
  sample.orientation_valid = true;
  sample.acceleration_valid = measurement.acceleration_valid;
  sample.angular_acceleration_valid = true;
  sample.valid = true;
  return update(sample);
}

HardwareImu::HardwareImu(std::string serial_device, int baudrate)
: serial_device_(std::move(serial_device)), baudrate_(baudrate)
{
  if (!serial_device_.empty()) {
    driver_ = std::make_unique<ImuDriver>(serial_device_, baudrate_);
  }
}

HardwareImu::~HardwareImu()
{
  close();
}

bool HardwareImu::open()
{
  if (serial_device_.empty()) {return false;}
  if (driver_ == nullptr) {
    driver_ = std::make_unique<ImuDriver>(serial_device_, baudrate_);
  }
  if (!driver_->start()) {return false;}
  if (startup_baseline_valid_) {return true;}
  if (calibrateStartupBaseline()) {return true;}
  driver_->stop();
  return false;
}

void HardwareImu::close() noexcept
{
  if (driver_ != nullptr) {driver_->stop();}
  startup_baseline_valid_ = false;
  startup_gravity_body_.setZero();
  startup_rpy_degrees_.setZero();
  startup_gravity_magnitude_ = 9.81F;
  startup_orientation_ = Eigen::Quaternionf::Identity();
}

bool HardwareImu::calibrateStartupBaseline()
{
  if (driver_ == nullptr) {return false;}

  constexpr std::size_t kMinimumSamples = 100;
  constexpr std::size_t kTargetSamples = 200;
  constexpr auto kCalibrationTimeout = std::chrono::seconds(3);
  constexpr float kMaximumStartupGyro = 0.15F;
  constexpr float kMinimumGravity = 1.0F;
  constexpr float kMaximumGravity = 20.0F;
  constexpr float kMaximumAccelerationStdDev = 0.5F;
  constexpr float kRadiansToDegrees = 57.29577951308232F;

  imu_log::print(
    imu_log::Level::Warning,
    "[DM IMU] collecting startup baseline; keep robot motionless...\n");
  const auto deadline = std::chrono::steady_clock::now() + kCalibrationTimeout;
  Vec3<float> acceleration_sum = Vec3<float>::Zero();
  Vec3<float> acceleration_square_sum = Vec3<float>::Zero();
  std::array<float, 3> rpy_sine_sum{};
  std::array<float, 3> rpy_cosine_sum{};
  std::chrono::steady_clock::time_point previous_received_at{};
  std::size_t sample_count = 0;

  while (std::chrono::steady_clock::now() < deadline &&
    sample_count < kTargetSamples)
  {
    DmImuRawSample raw_sample;
    if (driver_->latest(raw_sample, std::chrono::milliseconds(100)) &&
      raw_sample.received_at != previous_received_at)
    {
      previous_received_at = raw_sample.received_at;
      const Vec3<float> acceleration(
        raw_sample.acceleration[0], raw_sample.acceleration[1],
        raw_sample.acceleration[2]);
      const Vec3<float> angular_velocity(
        raw_sample.angular_velocity[0], raw_sample.angular_velocity[1],
        raw_sample.angular_velocity[2]);
      const bool finite = acceleration.allFinite() && angular_velocity.allFinite();
      const float acceleration_norm = acceleration.norm();
      const bool stationary = finite &&
        angular_velocity.norm() <= kMaximumStartupGyro &&
        acceleration_norm >= kMinimumGravity && acceleration_norm <= kMaximumGravity;
      if (stationary) {
        acceleration_sum += acceleration;
        acceleration_square_sum += acceleration.cwiseProduct(acceleration);
        for (int axis = 0; axis < 3; ++axis) {
          const float angle = raw_sample.rpy_degrees[axis] *
            0.01745329251994329577F;
          rpy_sine_sum[axis] += std::sin(angle);
          rpy_cosine_sum[axis] += std::cos(angle);
        }
        ++sample_count;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  if (sample_count < kMinimumSamples) {
    imu_log::print(
      imu_log::Level::Error,
      "[DM IMU] startup baseline rejected: only %zu stable samples; "
      "keep the robot still while IMU data is streaming.\n",
      sample_count);
    return false;
  }

  const float count = static_cast<float>(sample_count);
  const Vec3<float> average_acceleration = acceleration_sum / count;
  const Vec3<float> acceleration_variance = (
    acceleration_square_sum / count).cwiseProduct(Vec3<float>::Ones()) -
    average_acceleration.cwiseProduct(average_acceleration);
  const float maximum_std_dev = std::sqrt(std::max(0.0F, acceleration_variance.maxCoeff()));
  if (!std::isfinite(maximum_std_dev) || maximum_std_dev > kMaximumAccelerationStdDev) {
    imu_log::print(
      imu_log::Level::Error,
      "[DM IMU] startup baseline rejected: robot moved or acceleration is "
      "unstable (max std=%.3f m/s^2).\n",
      maximum_std_dev);
    return false;
  }

  Vec3<float> average_rpy_degrees = Vec3<float>::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    average_rpy_degrees[axis] = std::atan2(
      rpy_sine_sum[axis], rpy_cosine_sum[axis]) * kRadiansToDegrees;
  }
  const float gravity_magnitude = average_acceleration.norm();
  if (!std::isfinite(gravity_magnitude) || gravity_magnitude < kMinimumGravity) {
    imu_log::print(
      imu_log::Level::Error,
      "[DM IMU] startup baseline rejected: invalid gravity reference.\n");
    return false;
  }

  startup_gravity_body_ = average_acceleration;
  startup_gravity_magnitude_ = gravity_magnitude;
  startup_rpy_degrees_ = average_rpy_degrees;
  startup_orientation_ = rpyDegreesToQuaternion(startup_rpy_degrees_);
  startup_baseline_valid_ = true;
  imu_log::print(
    imu_log::Level::Info,
    "[DM IMU] startup baseline accepted: gravity_body=(%.3f, %.3f, %.3f) "
    "|g|=%.3f m/s^2 rpy0_deg=(%.3f, %.3f, %.3f) from %zu samples\n",
    startup_gravity_body_.x(), startup_gravity_body_.y(), startup_gravity_body_.z(),
    startup_gravity_magnitude_, startup_rpy_degrees_.x(), startup_rpy_degrees_.y(),
    startup_rpy_degrees_.z(), sample_count);
  return true;
}

bool HardwareImu::readAt(ImuData<float> & sample, float timestamp)
{
  sample = ImuData<float>{};
  if (!std::isfinite(timestamp) || serial_device_.empty()) {
    imu = sample;
    return false;
  }
  DmImuRawSample raw_sample;
  if (!startup_baseline_valid_ || driver_ == nullptr || !driver_->latest(raw_sample)) {
    imu = ImuData<float>{};
    sample = imu;
    return false;
  }
  const Vec3<float> current_rpy_degrees(
    raw_sample.rpy_degrees[0], raw_sample.rpy_degrees[1], raw_sample.rpy_degrees[2]);
  const Eigen::Quaternionf current_orientation =
    rpyDegreesToQuaternion(current_rpy_degrees);
  sample = ImuData<float>{};
  // 把启动时的 IMU 姿态作为局部世界系原点：R_relative = R0^-1 * R_current。
  sample.orientation_world_from_body = startup_orientation_.conjugate() * current_orientation;
  sample.angular_velocity_body <<
    raw_sample.angular_velocity[0], raw_sample.angular_velocity[1],
    raw_sample.angular_velocity[2];
  sample.acceleration_body <<
    raw_sample.acceleration[0], raw_sample.acceleration[1],
    raw_sample.acceleration[2];
  sample.timestamp = timestamp;
  sample.sequence = raw_sample.sequence;
  sample.orientation_valid = true;
  sample.acceleration_valid = true;
  sample.angular_acceleration_valid = false;
  sample.gravity_magnitude = startup_gravity_magnitude_;
  sample.gravity_valid = true;
  sample.valid = true;
  imu = sample;
  return true;
}

ImuData<float> HardwareImu::read()
{
  if (!serial_device_.empty()) {
    ImuData<float> sample;
    const float timestamp = std::chrono::duration<float>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    readAt(sample, timestamp);
    return sample;
  }
  // 无串口参数时保留同步注入模式，供仿真控制管线和单元测试使用。
  return imu;
}

// ---------- 工厂：按来源类型创建 IMU 数据源 ----------

std::unique_ptr<ImuSensor> makeImu(
  ImuSource source, const mjModel * model, const mjData * data)
{
  switch (source) {
    case ImuSource::SIMULATOR:
      return std::make_unique<SimImu>(model, data);
    case ImuSource::HARDWARE:
      return std::make_unique<HardwareImu>();
  }
  throw std::invalid_argument("unknown IMU source");
}
