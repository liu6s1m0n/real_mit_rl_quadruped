/*! @file imu.cpp
 *  @brief IMU 数据源实现。
 *
 *  真实硬件路径按型号创建串口接收驱动（DM / HFI-A9），本文件完成两者共用的
 *  启动基准、安装偏差补偿和数据转换；不再依赖独立的 ROS IMU 接收节点。
 */

#include "sensor/imu.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <utility>

#include "sensor/hfi_imu_driver.hpp"
#include "sensor/imu_driver.hpp"
#include "common/console_log.hpp"

// ---------- 内部辅助函数（匿名命名空间） ----------
namespace
{

/// 把 roll/pitch/yaw（度）按 ZYX 顺序转换为四元数。
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

// DM1 IMU 安装偏差默认值（传感器相对机体，单位度）。
// 来源：2026-11-01 22:18 / 22:19 与 2026-11-02 09:25 三次站立数据，
// 用加速度计重力方向对齐四足足端拟合的地面法向（假设地面水平），
// 三次拟合残差约 0.1°。修正前 IMU 读数机身水平，实际后低约 1.8°。
// 可用环境变量临时覆盖（无需重新编译）：
//   DM1_IMU_MOUNT_ROLL_DEG / DM1_IMU_MOUNT_PITCH_DEG / DM1_IMU_MOUNT_YAW_DEG
// 全部设为 0 即恢复旧行为（不做安装补偿）。
constexpr float kDefaultImuMountRollDeg = -1.32F;
constexpr float kDefaultImuMountPitchDeg = 1.84F;
constexpr float kDefaultImuMountYawDeg = 0.0F;
// HFI-A9 安装偏差默认值。来源：2026-11-06 同一位置 0° / 180° 两次静止站立
// （dm1_diag_imucal_20261106），用 scripts/imu_mount_calibration.py 把 IMU 竖直方向
// 对齐四足足端平面法向，两次平均消去地面坡度（两次差 ±0.2~0.3°）。
// 轴向已用倾斜测试确认与机体 FLU 对齐，yaw 无法由重力观测，保持 0。
constexpr float kDefaultHfiMountRollDeg = 0.97F;
constexpr float kDefaultHfiMountPitchDeg = -0.07F;
constexpr float kDefaultHfiMountYawDeg = 0.0F;
constexpr float kMaximumImuMountDeg = 10.0F;
// 下游估计器使用的标准重力；加速度计输出统一归一化到这个模长。
constexpr float kNominalGravity = 9.81F;
// 归一化系数的合理范围；超出说明启动基准异常，不做归一化。
constexpr float kMinimumAccelerationScale = 0.90F;
constexpr float kMaximumAccelerationScale = 1.10F;

/// 读取安装角环境变量；未设置、无法解析或超过 ±10° 时使用默认值。
float mountAngleFromEnvironment(const char * tag, const char * name, float default_value)
{
  const char * text = std::getenv(name);
  if (text == nullptr || *text == '\0') {return default_value;}
  char * end = nullptr;
  const float value = std::strtof(text, &end);
  if (end == text || !std::isfinite(value) || std::fabs(value) > kMaximumImuMountDeg) {
    imu_log::print(
      imu_log::Level::Warning,
      "%s 忽略无效的 %s='%s'（需为 |值|<=%.0f 的度数），使用默认 %.3f°。\n",
      tag, name, text, static_cast<double>(kMaximumImuMountDeg),
      static_cast<double>(default_value));
    return default_value;
  }
  return value;
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

// 读取一次 IMU 数据：从 MuJoCo 仿真数据中提取姿态、角速度、加速度和时间戳。
ImuData<float> SimImu::read() const
{
  const mjtNum * orientation = data_->sensordata + orientation_address_;
  const mjtNum * angular_velocity = data_->sensordata + angular_velocity_address_;
  const mjtNum * acceleration = data_->sensordata + acceleration_address_;

  ImuData<float> result;
  // 姿态：从机体系旋转到世界系的四元数 (w, x, y, z)
  result.orientation_world_from_body = Eigen::Quaternionf(
    orientation[0], orientation[1], orientation[2], orientation[3]);
  result.orientation_world_from_body.normalize();
  result.angular_velocity_body <<
    angular_velocity[0], angular_velocity[1], angular_velocity[2];
  result.acceleration_body <<
    acceleration[0], acceleration[1], acceleration[2];
  result.gravity_magnitude = 9.81F;
  result.timestamp = static_cast<float>(data_->time);
  result.valid = true;
  return result;
}

// 传感器查找与校验：按名称在 MuJoCo 模型中查找传感器，
// 校验其类型和维度，返回 sensordata 缓冲区中的起始地址。
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

  return model->sensor_adr[sensor_id];
}

// ---------- 真实硬件 IMU 数据源 ----------

// 构造时只读取安装偏差并创建对应型号的接收驱动，不打开串口。
HardwareImu::HardwareImu(std::string serial_device, int baudrate, ImuType type)
: serial_device_(std::move(serial_device)), baudrate_(baudrate), type_(type),
  log_tag_(std::string("[") + imuTypeName(type) + " IMU]")
{
  const bool hfi = type_ == ImuType::Hfi;
  const char * tag = log_tag_.c_str();
  mount_rpy_degrees_ = Vec3<float>(
    mountAngleFromEnvironment(
      tag, "DM1_IMU_MOUNT_ROLL_DEG", hfi ? kDefaultHfiMountRollDeg : kDefaultImuMountRollDeg),
    mountAngleFromEnvironment(
      tag, "DM1_IMU_MOUNT_PITCH_DEG", hfi ? kDefaultHfiMountPitchDeg : kDefaultImuMountPitchDeg),
    mountAngleFromEnvironment(
      tag, "DM1_IMU_MOUNT_YAW_DEG", hfi ? kDefaultHfiMountYawDeg : kDefaultImuMountYawDeg));
  mount_body_from_sensor_ = rpyDegreesToQuaternion(mount_rpy_degrees_);
  imu_log::print(
    imu_log::Level::Info,
    "%s 安装偏差补偿 mount_rpy_deg=(%.3f, %.3f, %.3f)（传感器相对机体）\n", tag,
    static_cast<double>(mount_rpy_degrees_.x()),
    static_cast<double>(mount_rpy_degrees_.y()),
    static_cast<double>(mount_rpy_degrees_.z()));
  if (hfi) {
    driver_ = std::make_unique<HfiImuDriver>(serial_device_, baudrate_);
  } else {
    driver_ = std::make_unique<ImuDriver>(serial_device_, baudrate_);
  }
}

HardwareImu::~HardwareImu()
{
  close();
}

// 启动接收驱动，并在静止状态下采集启动基准。
bool HardwareImu::open()
{
  if (!driver_->start()) {return false;}
  if (startup_baseline_valid_) {return true;}
  if (calibrateStartupBaseline()) {return true;}
  driver_->stop();
  return false;
}

// 停止接收驱动并清除启动基准。
void HardwareImu::close() noexcept
{
  driver_->stop();
  startup_baseline_valid_ = false;
  startup_gravity_body_.setZero();
  startup_rpy_degrees_.setZero();
  startup_gravity_magnitude_ = 9.81F;
  acceleration_scale_ = 1.0F;
  startup_heading_orientation_ = Eigen::Quaternionf::Identity();
}

bool HardwareImu::tryRecover()
{
  const Vec3<float> previous_gravity = startup_gravity_body_;
  const bool had_baseline = startup_baseline_valid_;
  // close() 会停止接收线程并清掉基准；open() 再用同一路径重开并重新标定。
  close();
  if (!open()) {
    static robot_log::Throttle retry_log(2000);
    if (retry_log.ready()) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 IMU 重连失败：设备 '%s' 仍不可用，或未通过启动静止标定；稍后重试。\n",
        serial_device_.c_str());
    }
    return false;
  }
  if (had_baseline) {
    const float dot = previous_gravity.normalized().dot(
      startup_gravity_body_.normalized());
    const float angle_deg =
      std::acos(std::clamp(dot, -1.0F, 1.0F)) * 57.29577951308232F;
    if (angle_deg > 8.0F) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 IMU 重连后重力基准偏差 %.1f°（机器人可能已移动）；"
        "姿态参考已按当前姿态重建，请确认机器人水平静止。\n",
        static_cast<double>(angle_deg));
    }
  }
  imu_log::print(
    imu_log::Level::Info,
    "DM1 IMU 重连成功：'%s' 已重新打开并重建启动基准。\n", serial_device_.c_str());
  return true;
}

// 采集 100~200 个静止样本（最长 3 s），求平均重力、启动姿态和加速度归一化系数。
// 角速度过大、重力模长异常或加速度波动过大的样本不计入。
bool HardwareImu::calibrateStartupBaseline()
{
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
    "%s collecting startup baseline; keep robot motionless...\n", log_tag_.c_str());
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
      "%s startup baseline rejected: only %zu stable samples; "
      "keep the robot still while IMU data is streaming.\n",
      log_tag_.c_str(), sample_count);
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
      "%s startup baseline rejected: robot moved or acceleration is "
      "unstable (max std=%.3f m/s^2).\n",
      log_tag_.c_str(), maximum_std_dev);
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
      "%s startup baseline rejected: invalid gravity reference.\n", log_tag_.c_str());
    return false;
  }

  // 姿态与加速度计一致性检查（只告警）：静止时 R_ws^T * z 应与实测比力同向。
  // 机身倾斜时可暴露欧拉角符号约定错误（例如 HFI 的 pitch 取反方向不对）。
  const Vec3<float> predicted_up_sensor =
    rpyDegreesToQuaternion(average_rpy_degrees).conjugate() * Vec3<float>::UnitZ();
  const float tilt_mismatch_degrees = std::acos(std::clamp(
      predicted_up_sensor.dot(average_acceleration / gravity_magnitude), -1.0F, 1.0F)) *
    kRadiansToDegrees;
  imu_log::print(
    tilt_mismatch_degrees > 3.0F ? imu_log::Level::Warning : imu_log::Level::Info,
    "%s 姿态/重力一致性：欧拉角推算的重力方向与加速度计相差 %.2f°%s\n",
    log_tag_.c_str(), static_cast<double>(tilt_mismatch_degrees),
    tilt_mismatch_degrees > 3.0F ? "（>3°，检查欧拉角符号约定或 IMU 是否在动）" : "");

  // 基准重力和启动姿态都转换到机体系，与 readAt() 的输出保持同一坐标语义。
  startup_gravity_body_ = mount_body_from_sensor_ * average_acceleration;
  startup_gravity_magnitude_ = gravity_magnitude;
  // 2026-11-02 静止测试：|g| 约 10.08 m/s^2，比 9.81 偏大约 2.8%，
  // 会在卡尔曼滤波里形成约 0.27 m/s^2 的持续向上虚假加速度，这里按启动模长归一化。
  const float acceleration_scale = kNominalGravity / gravity_magnitude;
  if (acceleration_scale >= kMinimumAccelerationScale &&
    acceleration_scale <= kMaximumAccelerationScale)
  {
    acceleration_scale_ = acceleration_scale;
  } else {
    acceleration_scale_ = 1.0F;
    imu_log::print(
      imu_log::Level::Warning,
      "%s 启动 |g|=%.3f m/s^2 超出合理范围，跳过加速度归一化（系数保持 1.0）。\n",
      log_tag_.c_str(), static_cast<double>(gravity_magnitude));
  }
  startup_rpy_degrees_ = average_rpy_degrees;
  // 加速度仍然是设备世界系中的重力向量，不能把启动 roll/pitch 一起减掉；
  // 只消除启动 yaw，既保持初始航向为零，又让姿态和重力方向使用同一坐标系。
  const Eigen::Matrix3f startup_world_from_body = (
    rpyDegreesToQuaternion(startup_rpy_degrees_) *
    mount_body_from_sensor_.conjugate()).toRotationMatrix();
  const float startup_body_yaw_degrees = std::atan2(
    startup_world_from_body(1, 0), startup_world_from_body(0, 0)) * kRadiansToDegrees;
  const Vec3<float> startup_heading_rpy(0.0F, 0.0F, startup_body_yaw_degrees);
  startup_heading_orientation_ = rpyDegreesToQuaternion(startup_heading_rpy);
  startup_baseline_valid_ = true;
  imu_log::print(
    imu_log::Level::Info,
    "%s startup baseline accepted: gravity_body=(%.3f, %.3f, %.3f) "
    "|g|=%.3f m/s^2 rpy0_deg=(%.3f, %.3f, %.3f) from %zu samples; "
    "acc_scale=%.4f\n",
    log_tag_.c_str(), startup_gravity_body_.x(), startup_gravity_body_.y(), startup_gravity_body_.z(),
    startup_gravity_magnitude_, startup_rpy_degrees_.x(), startup_rpy_degrees_.y(),
    startup_rpy_degrees_.z(), sample_count, static_cast<double>(acceleration_scale_));
  return true;
}

// 读取最新帧，去除启动航向并补偿安装偏差后输出到控制层。
bool HardwareImu::readAt(ImuData<float> & sample, float timestamp)
{
  sample = ImuData<float>{};
  // 逐个条件给出原因：上层只会看到"读取 IMU 或电机快照失败"，
  // 不区分清楚就无法判断是没做过静止标定、串口掉了还是线程停了。
  if (!startup_baseline_valid_) {
    static robot_log::Throttle baseline_throttle(1000);
    if (baseline_throttle.ready()) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 IMU 不可用：启动静止基准无效（标定未完成或已 close()）。\n");
    }
    return false;
  }
  DmImuRawSample raw_sample;
  if (!driver_->latest(raw_sample)) {
    static robot_log::Throttle stale_throttle(1000);
    if (stale_throttle.ready()) {
      imu_log::print(
        imu_log::Level::Warning,
        "DM1 IMU 不可用：没有新鲜帧（>50 ms 或接收线程已停止）；"
        "看上方 %s status=NO_DATA / INVALID_FRAME 提示。\n", log_tag_.c_str());
    }
    return false;
  }
  const Vec3<float> current_rpy_degrees(
    raw_sample.rpy_degrees[0], raw_sample.rpy_degrees[1], raw_sample.rpy_degrees[2]);
  const Eigen::Quaternionf current_orientation =
    rpyDegreesToQuaternion(current_rpy_degrees);
  // 只去除启动航向：R_input = R_yaw0^-1 * R_current。
  // 保留 roll/pitch，避免与未变换的机体系重力加速度产生坐标语义冲突。
  // 安装偏差补偿：R_wb = R_ws * R_bs^T；角速度和加速度从传感器系转到机体系。
  sample.orientation_world_from_body =
    startup_heading_orientation_.conjugate() * current_orientation *
    mount_body_from_sensor_.conjugate();
  sample.orientation_world_from_body.normalize();
  const Vec3<float> angular_velocity_sensor(
    raw_sample.angular_velocity[0], raw_sample.angular_velocity[1],
    raw_sample.angular_velocity[2]);
  const Vec3<float> acceleration_sensor(
    raw_sample.acceleration[0], raw_sample.acceleration[1],
    raw_sample.acceleration[2]);
  sample.angular_velocity_body = mount_body_from_sensor_ * angular_velocity_sensor;
  sample.acceleration_body =
    acceleration_scale_ * (mount_body_from_sensor_ * acceleration_sensor);
  sample.timestamp = timestamp;
  sample.sequence = raw_sample.sequence;
  sample.gravity_magnitude = startup_gravity_magnitude_;
  sample.valid = true;
  return true;
}
