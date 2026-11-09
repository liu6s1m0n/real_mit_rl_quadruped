/*! @file opendog_algorithm.cpp
 *  @brief OpenDogAlgorithm 实现：配置加载、MPC 后台线程和每周期的估计/WBC 计算。
 */
#include "control/opendog_algorithm.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <exception>
#include <fstream>
#include <stdexcept>
#include <utility>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_sqp/SqpMpc.h>

#include <legged_estimation_ros2/LinearKalmanFilter.hpp>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WeightedWbc.h>
#include "control/opendog_reference.hpp"

namespace
{

/// 去掉 YAML 标量两端的空白和成对引号。
std::string trimYamlScalar(std::string value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {return {};}
  const auto last = value.find_last_not_of(" \t\r\n");
  value = value.substr(first, last - first + 1);
  if (value.size() >= 2 &&
    ((value.front() == '"' && value.back() == '"') ||
    (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }
  return value;
}

/**
 * @brief 从 root/opendog.yaml 的 opendog: 节点读取配置文件路径。
 *
 * 这里只做最简单的逐行解析，不依赖 yaml-cpp。相对路径按 root 解析；
 * 找不到 key 或文件不存在时返回 root/fallback。
 */
std::filesystem::path configuredOpenDogFile(
  const std::filesystem::path & root, const std::string & key,
  const std::string & fallback)
{
  const auto yaml = root / "opendog.yaml";
  std::ifstream stream(yaml);
  bool in_opendog = false;
  std::string line;
  while (std::getline(stream, line)) {
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos || line[first] == '#') {continue;}
    const auto content = line.substr(first);
    if (content == "opendog:" || content.rfind("opendog:", 0) == 0) {
      in_opendog = true;
      continue;
    }
    // 回到顶格的其他键，说明 opendog: 节点已经结束。
    if (first == 0) {
      in_opendog = false;
      continue;
    }
    if (!in_opendog) {continue;}
    const auto marker = content.find(key + ":");
    if (marker != 0) {continue;}
    auto value = content.substr(key.size() + 1);
    const auto comment = value.find('#');
    if (comment != std::string::npos) {value.erase(comment);}
    value = trimYamlScalar(std::move(value));
    if (!value.empty()) {
      const std::filesystem::path configured(value);
      return configured.is_absolute() ? configured : root / configured;
    }
  }
  return root / fallback;
}

}  // namespace

/// OpenDogAlgorithm 的私有状态，持有全部 OCS2 求解组件。
struct OpenDogAlgorithm::Impl
{
  explicit Impl(std::string directory, double control_period)
  : config_directory(std::move(directory)), initial_period(control_period) {}

  std::string config_directory;                            ///< OpenDog 配置目录。
  Timing timing;                                           ///< 最近一周期的耗时统计。
  CommandSettings command_settings;                        ///< 从配置文件读取的命令参数。
  std::chrono::steady_clock::time_point last_timing_log{};  ///< 上次打印耗时日志的时间。
  double max_compute_ms = 0.0;                             ///< 日志窗口内的最大计算耗时。
  std::atomic<double> last_mpc_ms{0.0};                    ///< MPC 线程写、控制线程读的求解耗时。
  std::shared_ptr<rclcpp::Node> node;                      ///< 状态估计器内部需要的 ROS 节点。
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;  ///< 在 MPC 线程中 spin。
  std::thread mpc_thread;                                  ///< 后台 MPC 求解线程。
  std::atomic<bool> mpc_running{false};                    ///< MPC 线程运行标志。
  std::exception_ptr mpc_error;                            ///< MPC 线程抛出的异常，由控制线程重新抛出。
  std::shared_ptr<legged::LeggedInterface> legged_interface;  ///< 模型和最优控制问题。
  std::unique_ptr<ocs2::SqpMpc> mpc;                       ///< SQP-MPC 求解器。
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt;            ///< MPC 与实时控制之间的策略交换接口。
  std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> ee_kinematics;  ///< 足端运动学。
  std::shared_ptr<legged::KalmanFilterEstimate> state_estimate;         ///< 线性卡尔曼状态估计器。
  std::shared_ptr<ocs2::CentroidalModelRbdConversions> rbd_conversions;  ///< 刚体状态与质心状态互转。
  std::shared_ptr<legged::WeightedWbc> wbc;                ///< 加权全身控制器。
  std::unique_ptr<OpenDogReference> reference;             ///< 期望状态到目标轨迹的转换。
  std::shared_ptr<OpenDogGaitCommand> gait_command;        ///< 挂在 MPC 上的步态切换模块。
  double initial_period;                                   ///< 首帧估计使用的控制周期，s。
  float joint_kp = 4.0F;                                   ///< 关节 kp，可被 task 文件覆盖。
  float joint_kd = 5.0F;                                   ///< 关节 kd，可被 task 文件覆盖。
  /// 各腿小腿（KFE）单独的 kp，按 FR/FL/RR/RL 存放，缺省等于 joint_kp。
  std::array<float, kNumLegs> calf_kp{{4.0F, 4.0F, 4.0F, 4.0F}};
  /// OCS2 质心模型观测：state 依次为归一化质心动量(6)、机身位置(3)、
  /// ZYX 欧拉角 yaw/pitch/roll(3)、12 个关节角。
  ocs2::SystemObservation observation;
  ocs2::vector_t measured_rbd_state;                       ///< 估计器输出的刚体状态。
  double time = 0.0;                                       ///< 上一次 compute() 的时间，s。
  bool initialized = false;                                ///< 是否已拿到初始策略并启动 MPC 线程。
  bool configured = false;                                 ///< configure() 是否成功。
  /// 本工程腿序 FR/FL/RR/RL 到 OpenDog URDF 腿序 LF/LH/RF/RH 的映射。
  std::array<std::size_t, kNumLegs> opendog_leg_for_current{{2, 0, 3, 1}};
  std::uint64_t command_sequence = 0;                      ///< 输出命令序号。
};

OpenDogAlgorithm::OpenDogAlgorithm(std::string config_directory, double control_period)
: impl_(std::make_unique<Impl>(std::move(config_directory), control_period))
{}

OpenDogAlgorithm::~OpenDogAlgorithm() {stop();}

void OpenDogAlgorithm::setMotionRequest(teleop::Motion motion, std::uint64_t sequence)
{
  if (impl_->gait_command) {impl_->gait_command->setMotion(motion, sequence);}
}

const OpenDogAlgorithm::Timing & OpenDogAlgorithm::timing() const noexcept
{
  return impl_->timing;
}

const OpenDogAlgorithm::CommandSettings & OpenDogAlgorithm::commandSettings() const noexcept
{
  return impl_->command_settings;
}

bool OpenDogAlgorithm::configure()
{
  if (impl_->configured || !rclcpp::ok()) {return impl_->configured;}
  const std::filesystem::path root(impl_->config_directory);
  const auto task = configuredOpenDogFile(root, "task_file", "opendog_task.info").string();
  const auto reference = configuredOpenDogFile(
    root, "reference_file", "opendog_reference.info").string();
  const auto urdf = configuredOpenDogFile(root, "urdf_file", "dm1_opendog.urdf").string();
  const auto gait = configuredOpenDogFile(root, "gait_file", "opendog_gait.info").string();
  if (!std::filesystem::exists(task) || !std::filesystem::exists(reference) ||
    !std::filesystem::exists(urdf) || !std::filesystem::exists(gait)) {return false;}

  impl_->node = rclcpp::Node::make_shared("dm1_opendog_algorithm");
  // 读 URDF 建 Pinocchio 模型，组装最优控制问题（代价、约束、动力学）
  impl_->legged_interface = std::make_shared<legged::LeggedInterface>(task, urdf, reference);
  impl_->legged_interface->setupOptimalControlProblem(task, urdf, reference, false);
  
  ocs2::CentroidalModelPinocchioMapping mapping(
    impl_->legged_interface->getCentroidalModelInfo());
  impl_->ee_kinematics = std::make_shared<ocs2::PinocchioEndEffectorKinematics>(
    impl_->legged_interface->getPinocchioInterface(), mapping,
    impl_->legged_interface->modelSettings().contactNames3DoF);
  // 状态估计器、WBC、MPC、参考轨迹和步态命令
  impl_->state_estimate = std::make_shared<legged::KalmanFilterEstimate>(
    impl_->legged_interface->getPinocchioInterface(),
    impl_->legged_interface->getCentroidalModelInfo(), *impl_->ee_kinematics,
    impl_->node);
  impl_->state_estimate->loadSettings(task, false);
  // 加权 WBC
  impl_->wbc = std::make_shared<legged::WeightedWbc>(
    impl_->legged_interface->getPinocchioInterface(),
    impl_->legged_interface->getCentroidalModelInfo(), *impl_->ee_kinematics);
  impl_->wbc->loadTasksSetting(task, false);
  // 刚体状态与质心模型状态之间的转换
  impl_->rbd_conversions = std::make_shared<ocs2::CentroidalModelRbdConversions>(
    impl_->legged_interface->getPinocchioInterface(),
    impl_->legged_interface->getCentroidalModelInfo());
  // SQP-MPC 求解器
  impl_->mpc = std::make_unique<ocs2::SqpMpc>(
    impl_->legged_interface->mpcSettings(), impl_->legged_interface->sqpSettings(),
    impl_->legged_interface->getOptimalControlProblem(),
    impl_->legged_interface->getInitializer());
  impl_->mpc->getSolverPtr()->setReferenceManager(
    impl_->legged_interface->getReferenceManagerPtr());
  // 把速度指令变成目标轨迹
  impl_->reference = std::make_unique<OpenDogReference>(reference, task);
  // 切换步态，并挂到 MPC 求解器上，在每次求解前同步插入
  impl_->gait_command = std::make_shared<OpenDogGaitCommand>(
    gait,
    impl_->legged_interface->getSwitchedModelReferenceManagerPtr()->getGaitSchedule());
  impl_->mpc->getSolverPtr()->addSynchronizedModule(impl_->gait_command);
  // 电机 PD 增益；小腿 kp 可按腿单独配置，缺省回落到 joint_kp
  ocs2::loadData::loadCppDataType(task, "hardware_output.kp", impl_->joint_kp);
  ocs2::loadData::loadCppDataType(task, "hardware_output.kd", impl_->joint_kd);
  boost::property_tree::ptree output_settings;
  boost::property_tree::read_info(task, output_settings);
  constexpr std::array<const char *, kNumLegs> calf_kp_keys{
    "hardware_output.FR_calf_kp", "hardware_output.FL_calf_kp",
    "hardware_output.RR_calf_kp", "hardware_output.RL_calf_kp"};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    impl_->calf_kp[leg] = output_settings.get<float>(calf_kp_keys[leg], impl_->joint_kp);
  }
  // 站立高度和键盘速度档位
  ocs2::loadData::loadCppDataType(reference, "comHeight", impl_->command_settings.standing_height);
  auto & velocity = impl_->command_settings.velocity;
  velocity.forward = output_settings.get<float>("hardware_command.forward", velocity.forward);
  velocity.forward_fast = output_settings.get<float>("hardware_command.forward_fast", velocity.forward_fast);
  velocity.backward = output_settings.get<float>("hardware_command.backward", velocity.backward);
  velocity.lateral = output_settings.get<float>("hardware_command.lateral", velocity.lateral);
  velocity.yaw = output_settings.get<float>("hardware_command.yaw_rate", velocity.yaw);
  impl_->command_settings.joint_kp = impl_->joint_kp;
  impl_->command_settings.joint_kd = impl_->joint_kd;
  std::printf(
    "[OpenDog][OUTPUT] kp=%.3f kd=%.3f calf_kp=[%.3f %.3f %.3f %.3f]\n",
    impl_->joint_kp, impl_->joint_kd, impl_->calf_kp[0], impl_->calf_kp[1],
    impl_->calf_kp[2], impl_->calf_kp[3]);
  // MRT 负责在控制线程和 MPC 线程之间交换观测与策略
  impl_->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  impl_->executor->add_node(impl_->node);
  impl_->mrt = std::make_unique<ocs2::MPC_MRT_Interface>(*impl_->mpc);
  impl_->mrt->initRollout(&impl_->legged_interface->getRollout());
  impl_->observation.state.setZero(impl_->legged_interface->getCentroidalModelInfo().stateDim);
  impl_->observation.input.setZero(impl_->legged_interface->getCentroidalModelInfo().inputDim);
  impl_->configured = true;
  return true;
}

bool OpenDogAlgorithm::compute(
  const ControlInputSnapshot & input, ControlOutput & output)
{
  if (!impl_->configured || !input.imu.valid) {
    output.valid = false;
    return false;
  }
  if (impl_->initialized && !impl_->mpc_running.load() && impl_->mpc_error) {
    std::rethrow_exception(impl_->mpc_error);
  }

  // 1. 关节反馈按腿序映射写入 OpenDog 关节向量
  const auto compute_start = std::chrono::steady_clock::now();
  impl_->timing.initial_policy = !impl_->initialized;
  const auto & info = impl_->legged_interface->getCentroidalModelInfo();
  ocs2::vector_t joint_position(info.actuatedDofNum);
  ocs2::vector_t joint_velocity(info.actuatedDofNum);
  for (std::size_t current_leg = 0; current_leg < kNumLegs; ++current_leg) {
    const std::size_t opendog_leg = impl_->opendog_leg_for_current[current_leg];
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::size_t opendog_index = opendog_leg * kJointsPerLeg + joint;
      joint_position(static_cast<Eigen::Index>(opendog_index)) =
        input.joints[current_leg].position(static_cast<Eigen::Index>(joint));
      joint_velocity(static_cast<Eigen::Index>(opendog_index)) =
        input.joints[current_leg].velocity(static_cast<Eigen::Index>(joint));
    }
  }

  // 2. 状态估计：关节、计划接触状态和 IMU 一起送入卡尔曼滤波
  impl_->state_estimate->updateJointStates(joint_position, joint_velocity);
  const auto mode = impl_->initialized ?
    impl_->mrt->getPolicy().modeSchedule_.modeAtTime(impl_->time) :
    static_cast<std::size_t>(ocs2::legged_robot::ModeNumber::STANCE);
  impl_->state_estimate->updateContact(ocs2::legged_robot::modeNumber2StanceLeg(mode));
  const auto quat = input.imu.orientation_world_from_body.cast<ocs2::scalar_t>();
  const auto angular = input.imu.angular_velocity_body.cast<ocs2::scalar_t>();
  const auto acceleration = input.imu.acceleration_body.cast<ocs2::scalar_t>();
  const ocs2::legged_robot::matrix3_t covariance =
    ocs2::legged_robot::matrix3_t::Zero();
  const auto ros_time = rclcpp::Time(static_cast<std::int64_t>(input.timestamp * 1e9));
  const auto period = rclcpp::Duration::from_seconds(
    impl_->initialized ? input.timestamp - impl_->time : impl_->initial_period);
  impl_->state_estimate->updateImu(quat, angular, acceleration, covariance, covariance, covariance);
  impl_->measured_rbd_state = impl_->state_estimate->update(ros_time, period);
  impl_->time = input.timestamp;
  impl_->observation.time = impl_->time;
  // yaw 做展开处理，避免跨越 ±pi 时观测跳变 2pi
  const auto yaw_last = impl_->observation.state(9);
  impl_->observation.state = impl_->rbd_conversions->computeCentroidalStateFromRbdModel(
    impl_->measured_rbd_state);
  const auto yaw_delta = impl_->observation.state(9) - yaw_last;
  impl_->observation.state(9) = yaw_last + std::atan2(std::sin(yaw_delta), std::cos(yaw_delta));
  impl_->observation.mode = impl_->state_estimate->getMode();

  // 3. 期望状态快照是唯一的命令边界：直接更新 OpenDog 参考管理器，
  // 不再经过原工程的 controller_manager 硬件插件转发命令。
  impl_->mrt->setCurrentObservation(impl_->observation);
  impl_->mrt->getReferenceManager().setTargetTrajectories(
    impl_->reference->update(input.desired, impl_->observation));
  impl_->gait_command->setMode(input.desired.mode);

  // 4. 首帧同步求出初始策略，然后启动后台 MPC 线程
  if (!impl_->initialized) {
    impl_->observation.input.setZero();
    while (!impl_->mrt->initialPolicyReceived()) {
      const auto solve_start = std::chrono::steady_clock::now();
      impl_->mrt->advanceMpc();
      impl_->last_mpc_ms.store(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - solve_start).count());
    }
    impl_->initialized = true;
    impl_->mpc_error = nullptr;
    impl_->mpc_running.store(true);
    impl_->mpc_thread = std::thread([this]() {
        try {
          while (impl_->mpc_running.load()) {
            ocs2::executeAndSleep([this]() {
                impl_->executor->spin_some();
                const auto solve_start = std::chrono::steady_clock::now();
                impl_->mrt->advanceMpc();
                impl_->last_mpc_ms.store(std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - solve_start).count());
              }, impl_->legged_interface->mpcSettings().mpcDesiredFrequency_);
          }
        } catch (const std::exception &) {
          // 异常交给控制线程在下一周期重新抛出
          impl_->mpc_error = std::current_exception();
          impl_->mpc_running.store(false);
        }
      });
    ocs2::setThreadPriority(
      impl_->legged_interface->sqpSettings().threadPriority, impl_->mpc_thread);
  }
  // 5. 取最新策略插值得到当前时刻的最优状态/输入，再由 WBC 求关节力矩
  impl_->mrt->updatePolicy();

  ocs2::vector_t optimized_state;
  ocs2::vector_t optimized_input;
  std::size_t planned_mode = 0;
  impl_->mrt->evaluatePolicy(
    impl_->observation.time, impl_->observation.state,
    optimized_state, optimized_input, planned_mode);
  impl_->observation.input = optimized_input;
  const auto wbc_start = std::chrono::steady_clock::now();
  const auto wbc_solution = impl_->wbc->update(
    optimized_state, optimized_input, impl_->measured_rbd_state,
    planned_mode, period.seconds());
  impl_->timing.wbc_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - wbc_start).count();
  const auto torque = wbc_solution.tail(static_cast<Eigen::Index>(kNumJoints));
  const auto position = ocs2::centroidal_model::getJointAngles(
    optimized_state, info);
  const auto velocity = ocs2::centroidal_model::getJointVelocities(
    optimized_input, info);

  // 6. 填写状态估计输出；世界系加速度减去重力
  output.valid = true;
  auto & estimate = output.estimate;
  estimate.orientation_world_from_body = input.imu.orientation_world_from_body;
  estimate.rotation_world_from_body = estimate.orientation_world_from_body.toRotationMatrix();
  estimate.rpy << float(impl_->observation.state(11)), float(impl_->observation.state(10)),
    float(impl_->observation.state(9));
  estimate.position_world = impl_->measured_rbd_state.segment<3>(3).cast<float>();
  estimate.velocity_world = impl_->measured_rbd_state.segment<3>(
    info.generalizedCoordinatesNum + 3).cast<float>();
  estimate.velocity_body = estimate.rotation_world_from_body.transpose() * estimate.velocity_world;
  estimate.angular_velocity_body = input.imu.angular_velocity_body;
  estimate.acceleration_body = input.imu.acceleration_body;
  estimate.acceleration_world = estimate.rotation_world_from_body * estimate.acceleration_body;
  estimate.acceleration_world.z() -= 9.81F;
  estimate.timestamp = input.timestamp;
  estimate.valid = true;
  // 7. 按腿序映射回本工程顺序，生成 PD+前馈关节命令
  ++impl_->command_sequence;
  for (std::size_t current_leg = 0; current_leg < kNumLegs; ++current_leg) {
    const std::size_t opendog_leg = impl_->opendog_leg_for_current[current_leg];
    auto & command = output.commands[current_leg];
    command.leg = static_cast<LegId>(current_leg);
    command.timestamp = static_cast<float>(input.timestamp);
    command.sequence = impl_->command_sequence;
    command.kp.setConstant(impl_->joint_kp);
    command.kp[2] = impl_->calf_kp[current_leg];
    command.kd.setConstant(impl_->joint_kd);
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::size_t opendog_index = opendog_leg * kJointsPerLeg + joint;
      command.position_desired(static_cast<Eigen::Index>(joint)) =
        static_cast<float>(position(static_cast<Eigen::Index>(opendog_index)));
      command.velocity_desired(static_cast<Eigen::Index>(joint)) =
        static_cast<float>(velocity(static_cast<Eigen::Index>(opendog_index)));
      command.torque_feedforward(static_cast<Eigen::Index>(joint)) =
        static_cast<float>(torque(static_cast<Eigen::Index>(opendog_index)));
    }
    command.enabled = true;
  }
  // 7b. 求解器诊断，仅供日志：接触按 WBC 实际使用的计划模式，
  // MPC 足端力取自最优输入，WBC 足端力取自解向量中广义加速度之后的 3×4 段
  auto & diagnostics = output.opendog;
  diagnostics.active_mode = mode;
  diagnostics.planned_mode = planned_mode;
  const auto stance_legs = ocs2::legged_robot::modeNumber2StanceLeg(planned_mode);
  const auto wbc_force_offset = static_cast<Eigen::Index>(info.generalizedCoordinatesNum);
  for (std::size_t current_leg = 0; current_leg < kNumLegs; ++current_leg) {
    const std::size_t opendog_leg = impl_->opendog_leg_for_current[current_leg];
    diagnostics.contact[current_leg] = stance_legs[opendog_leg];
    const auto force_des = ocs2::centroidal_model::getContactForces(
      optimized_input, opendog_leg, info);
    const auto force_wbc = wbc_solution.segment<3>(
      wbc_force_offset + static_cast<Eigen::Index>(3 * opendog_leg));
    for (std::size_t axis = 0; axis < 3; ++axis) {
      diagnostics.force_des[current_leg][axis] =
        static_cast<float>(force_des(static_cast<Eigen::Index>(axis)));
      diagnostics.force_wbc[current_leg][axis] =
        static_cast<float>(force_wbc(static_cast<Eigen::Index>(axis)));
    }
  }
  diagnostics.valid = true;
  // 8. 耗时统计，每秒打印一次
  auto & timing = impl_->timing;
  const auto now = std::chrono::steady_clock::now();
  timing.compute_ms = std::chrono::duration<double, std::milli>(now - compute_start).count();
  timing.mpc_ms = impl_->last_mpc_ms.load();
  timing.policy_age_ms = 1000.0 *
    (impl_->observation.time - impl_->mrt->getCommand().mpcInitObservation_.time);
  timing.horizon_remaining_ms = 1000.0 *
    (impl_->mrt->getPolicy().timeTrajectory_.back() - impl_->observation.time);
  timing.planned_mode = planned_mode;
  impl_->max_compute_ms = std::max(impl_->max_compute_ms, timing.compute_ms);
  if (now - impl_->last_timing_log >= std::chrono::seconds(1)) {
    std::printf("[OpenDog][TIMING] compute_ms=%.3f max_compute_ms=%.3f wbc_ms=%.3f "
      "mpc_ms=%.3f policy_age_ms=%.3f horizon_remaining_ms=%.3f monitor_only=true\n",
      timing.compute_ms, impl_->max_compute_ms, timing.wbc_ms, timing.mpc_ms,
      timing.policy_age_ms, timing.horizon_remaining_ms);
    impl_->last_timing_log = now;
    impl_->max_compute_ms = 0.0;
  }
  return true;
}

void OpenDogAlgorithm::stop() noexcept
{
  impl_->mpc_running.store(false);
  if (impl_->mpc_thread.joinable()) {impl_->mpc_thread.join();}
  if (impl_->mrt) {impl_->mrt->reset();}
  if (impl_->mpc) {impl_->mpc->reset();}
  if (impl_->reference) {impl_->reference->reset();}
  if (impl_->gait_command) {impl_->gait_command->reset();}
  impl_->initialized = false;
}

void OpenDogAlgorithm::reset() noexcept
{
  stop();
  // 再次使能（Shift+U）后会重新站立：状态估计器和 WBC 也必须回到初始状态。
  // 否则估计器仍保留上一次站立结束时约 0.27 m 的机身高度，而机器人实际趴在地上，
  // 第一帧 MPC/WBC 会给出约 30 Nm 的力矩尖峰和约 0.2 rad 的期望关节跳变（11/02 实测）。
  if (impl_->state_estimate) {impl_->state_estimate->reset();}
  if (impl_->wbc) {impl_->wbc->reset();}
  impl_->time = 0.0;
  impl_->observation.state.setZero();
  impl_->observation.input.setZero();
}
