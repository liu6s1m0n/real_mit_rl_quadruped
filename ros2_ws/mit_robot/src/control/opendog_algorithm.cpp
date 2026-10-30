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

#if defined(MYMIT_ENABLE_OPENDOG)
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
#endif

namespace
{

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

struct OpenDogAlgorithm::Impl
{
  explicit Impl(std::string directory, double control_period)
  : config_directory(std::move(directory)), initial_period(control_period) {}

  std::string config_directory;
  Timing timing;
  CommandSettings command_settings;
  std::chrono::steady_clock::time_point last_timing_log{};
  double max_compute_ms = 0.0;
  std::atomic<double> last_mpc_ms{0.0};
#if defined(MYMIT_ENABLE_OPENDOG)
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread mpc_thread;
  std::atomic<bool> mpc_running{false};
  std::atomic<bool> initial_policy_ready{false};
  ocs2::vector_t initial_state;
  std::exception_ptr mpc_error;
  std::shared_ptr<legged::LeggedInterface> legged_interface;
  std::unique_ptr<ocs2::SqpMpc> mpc;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mrt;
  std::shared_ptr<ocs2::PinocchioEndEffectorKinematics> ee_kinematics;
  std::shared_ptr<legged::KalmanFilterEstimate> state_estimate;
  std::shared_ptr<ocs2::CentroidalModelRbdConversions> rbd_conversions;
  std::shared_ptr<legged::WeightedWbc> wbc;
  std::unique_ptr<OpenDogReference> reference;
  std::shared_ptr<OpenDogGaitCommand> gait_command;
  double initial_period;
  float joint_kp = 4.0F;
  float joint_kd = 5.0F;
  ocs2::SystemObservation observation;
  ocs2::vector_t measured_rbd_state;
  double time = 0.0;
  bool initialized = false;
  bool configured = false;
  std::array<std::size_t, kNumLegs> opendog_leg_for_current{{2, 0, 3, 1}};
  std::uint64_t command_sequence = 0;
  ocs2::vector_t last_position_command;
  ocs2::vector_t last_velocity_command;
  ocs2::vector_t last_torque_command;
  bool have_last_command = false;
  std::size_t last_active_mode = static_cast<std::size_t>(
    ocs2::legged_robot::ModeNumber::STANCE);
  std::size_t last_planned_mode = static_cast<std::size_t>(
    ocs2::legged_robot::ModeNumber::STANCE);
  double handoff_start_time = 0.0;
  ocs2::vector_t handoff_position;
  ocs2::vector_t handoff_velocity;
  ocs2::vector_t handoff_torque;
  bool mode_handoff_active = false;
#endif
};

OpenDogAlgorithm::OpenDogAlgorithm(std::string config_directory, double control_period)
: impl_(std::make_unique<Impl>(std::move(config_directory), control_period))
{}

OpenDogAlgorithm::~OpenDogAlgorithm() {stop();}

void OpenDogAlgorithm::setMotionRequest(teleop::Motion motion, std::uint64_t sequence)
{
#if defined(MYMIT_ENABLE_OPENDOG)
  if (impl_->gait_command) {impl_->gait_command->setMotion(motion, sequence);}
#else
  (void)motion;
  (void)sequence;
#endif
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
#if !defined(MYMIT_ENABLE_OPENDOG)
  return false;
#else
  if (impl_->configured || !rclcpp::ok()) {return impl_->configured;}
  const std::filesystem::path root(impl_->config_directory);
  const auto task = configuredOpenDogFile(root, "task_file", "opendog_task.info").string();
  const auto reference = configuredOpenDogFile(
    root, "reference_file", "opendog_reference.info").string();
  const auto urdf = configuredOpenDogFile(root, "urdf_file", "dm1_opendog.urdf").string();
  const auto gait = configuredOpenDogFile(root, "gait_file", "opendog_gait.info").string();
  if (!std::filesystem::exists(task) || !std::filesystem::exists(reference) ||
    !std::filesystem::exists(urdf) || !std::filesystem::exists(gait)) {return false;}

  impl_->node = rclcpp::Node::make_shared("mymit_opendog_algorithm");
  impl_->legged_interface = std::make_shared<legged::LeggedInterface>(task, urdf, reference);
  impl_->legged_interface->setupOptimalControlProblem(task, urdf, reference, false);

  ocs2::CentroidalModelPinocchioMapping mapping(
    impl_->legged_interface->getCentroidalModelInfo());
  impl_->ee_kinematics = std::make_shared<ocs2::PinocchioEndEffectorKinematics>(
    impl_->legged_interface->getPinocchioInterface(), mapping,
    impl_->legged_interface->modelSettings().contactNames3DoF);
  impl_->state_estimate = std::make_shared<legged::KalmanFilterEstimate>(
    impl_->legged_interface->getPinocchioInterface(),
    impl_->legged_interface->getCentroidalModelInfo(), *impl_->ee_kinematics,
    impl_->node);
  impl_->state_estimate->loadSettings(task, false);
  impl_->wbc = std::make_shared<legged::WeightedWbc>(
    impl_->legged_interface->getPinocchioInterface(),
    impl_->legged_interface->getCentroidalModelInfo(), *impl_->ee_kinematics);
  impl_->wbc->loadTasksSetting(task, false);
  impl_->rbd_conversions = std::make_shared<ocs2::CentroidalModelRbdConversions>(
    impl_->legged_interface->getPinocchioInterface(),
    impl_->legged_interface->getCentroidalModelInfo());
  impl_->mpc = std::make_unique<ocs2::SqpMpc>(
    impl_->legged_interface->mpcSettings(), impl_->legged_interface->sqpSettings(),
    impl_->legged_interface->getOptimalControlProblem(),
    impl_->legged_interface->getInitializer());
  impl_->mpc->getSolverPtr()->setReferenceManager(
    impl_->legged_interface->getReferenceManagerPtr());
  impl_->reference = std::make_unique<OpenDogReference>(reference, task);
  impl_->gait_command = std::make_shared<OpenDogGaitCommand>(
    gait,
    impl_->legged_interface->getSwitchedModelReferenceManagerPtr()->getGaitSchedule());
  impl_->mpc->getSolverPtr()->addSynchronizedModule(impl_->gait_command);
  ocs2::loadData::loadCppDataType(task, "hardware_output.kp", impl_->joint_kp);
  ocs2::loadData::loadCppDataType(task, "hardware_output.kd", impl_->joint_kd);
  boost::property_tree::ptree output_settings;
  boost::property_tree::read_info(task, output_settings);
  ocs2::loadData::loadCppDataType(reference, "comHeight", impl_->command_settings.standing_height);
  auto & velocity = impl_->command_settings.velocity;
  velocity.forward = output_settings.get<float>("hardware_command.forward", velocity.forward);
  velocity.forward_fast = output_settings.get<float>("hardware_command.forward_fast", velocity.forward_fast);
  velocity.backward = output_settings.get<float>("hardware_command.backward", velocity.backward);
  velocity.lateral = output_settings.get<float>("hardware_command.lateral", velocity.lateral);
  velocity.yaw = output_settings.get<float>("hardware_command.yaw_rate", velocity.yaw);
  impl_->command_settings.joint_kp = impl_->joint_kp;
  impl_->command_settings.joint_kd = impl_->joint_kd;
  std::printf("[OpenDog][OUTPUT] kp=%.3f kd=%.3f\n",
    impl_->joint_kp, impl_->joint_kd);
  impl_->legged_interface->getSwitchedModelReferenceManagerPtr()->attachRosNode(impl_->node);
  impl_->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  impl_->executor->add_node(impl_->node);
  impl_->mrt = std::make_unique<ocs2::MPC_MRT_Interface>(*impl_->mpc);
  impl_->mrt->initRollout(&impl_->legged_interface->getRollout());
  impl_->observation.state.setZero(impl_->legged_interface->getCentroidalModelInfo().stateDim);
  impl_->observation.input.setZero(impl_->legged_interface->getCentroidalModelInfo().inputDim);
  impl_->configured = true;
  return true;
#endif
}

bool OpenDogAlgorithm::start()
{
#if !defined(MYMIT_ENABLE_OPENDOG)
  return false;
#else
  return impl_->configured;
#endif
}

bool OpenDogAlgorithm::compute(
  const ControlInputSnapshot & input, ControlOutput & output)
{
#if !defined(MYMIT_ENABLE_OPENDOG)
  (void)input;
  output.backend = AlgorithmBackendId::OpenDog;
  output.valid = false;
  return false;
#else
  if (!impl_->configured || !impl_->mrt || !impl_->wbc || !input.imu.valid) {
    output.backend = AlgorithmBackendId::OpenDog;
    output.valid = false;
    return false;
  }
  const bool mpc_warning = impl_->initialized && !impl_->mpc_running.load() && impl_->mpc_error;

  const auto compute_start = std::chrono::steady_clock::now();
  const bool policy_ready = impl_->initial_policy_ready.load();
  impl_->timing.initial_policy = !policy_ready;
  if (policy_ready) {impl_->mrt->updatePolicy();}
  // Keep one Locomotion request path for trot and one-shot diagonal steps.
  // The synchronized module decides whether the request is continuous trot
  // or a finite step; this call does not alter the output command.
  impl_->gait_command->setMode(input.desired.mode);
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

  impl_->state_estimate->updateJointStates(joint_position, joint_velocity);
  const std::size_t active_mode = impl_->initialized && !impl_->timing.initial_policy ?
    impl_->mrt->getPolicy().modeSchedule_.modeAtTime(input.timestamp) :
    static_cast<std::size_t>(ocs2::legged_robot::ModeNumber::STANCE);
  impl_->state_estimate->updateContact(
    ocs2::legged_robot::modeNumber2StanceLeg(active_mode));
  const auto quat = input.imu.orientation_world_from_body.cast<ocs2::scalar_t>();
  const auto angular = input.imu.angular_velocity_body.cast<ocs2::scalar_t>();
  // The KF subtracts 9.81 internally. Align the measured static gravity with
  // that convention without scaling dynamic acceleration or model gravity.
  const ocs2::scalar_t gravity_reference = input.imu.gravity_valid ?
    static_cast<ocs2::scalar_t>(input.imu.gravity_magnitude) : 9.81;
  const ocs2::legged_robot::vector3_t acceleration =
    input.imu.acceleration_body.cast<ocs2::scalar_t>() -
    quat.conjugate() * ocs2::legged_robot::vector3_t(0.0, 0.0, gravity_reference - 9.81);
  const ocs2::legged_robot::matrix3_t covariance =
    ocs2::legged_robot::matrix3_t::Zero();
  const auto ros_time = rclcpp::Time(static_cast<std::int64_t>(input.timestamp * 1e9));
  const auto period = rclcpp::Duration::from_seconds(
    impl_->initialized ? input.timestamp - impl_->time : impl_->initial_period);
  impl_->state_estimate->updateImu(quat, angular, acceleration, covariance, covariance, covariance);
  impl_->measured_rbd_state = impl_->state_estimate->update(ros_time, period);
  impl_->time = input.timestamp;
  impl_->observation.time = impl_->time;
  const auto yaw_last = impl_->observation.state(9);
  impl_->observation.state = impl_->rbd_conversions->computeCentroidalStateFromRbdModel(
    impl_->measured_rbd_state);
  const auto yaw_delta = impl_->observation.state(9) - yaw_last;
  impl_->observation.state(9) = yaw_last + std::atan2(std::sin(yaw_delta), std::cos(yaw_delta));
  impl_->observation.mode = impl_->state_estimate->getMode();

  impl_->mrt->setCurrentObservation(impl_->observation);
  if (!impl_->initialized) {
    // LeggedController::starting(): first solve holds the measured state.
    // Run it on the existing MPC worker so hardware keeps its 2 ms cycle.
    impl_->observation.input.setZero();
    impl_->initial_state = impl_->observation.state;
    impl_->mrt->getReferenceManager().setTargetTrajectories(
      {{impl_->observation.time}, {impl_->initial_state}, {impl_->observation.input}});
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
                impl_->initial_policy_ready.store(impl_->mrt->initialPolicyReceived());
              }, impl_->legged_interface->mpcSettings().mpcDesiredFrequency_);
          }
        } catch (const std::exception &) {
          impl_->mpc_error = std::current_exception();
          impl_->mpc_running.store(false);
        }
      });
    ocs2::setThreadPriority(
      impl_->legged_interface->sqpSettings().threadPriority, impl_->mpc_thread);
  }
  ocs2::vector_t optimized_state = impl_->initial_state;
  ocs2::vector_t optimized_input = ocs2::vector_t::Zero(info.inputDim);
  ocs2::vector_t torque = ocs2::vector_t::Zero(kNumJoints);
  ocs2::vector_t wbc_solution;
  std::size_t planned_mode = ocs2::legged_robot::ModeNumber::STANCE;
  impl_->timing.wbc_ms = 0.0;
  if (policy_ready) {
    // Start the requested trajectory only after the initial holding policy.
    impl_->mrt->getReferenceManager().setTargetTrajectories(
      impl_->reference->update(input.desired, impl_->observation));
    impl_->mrt->evaluatePolicy(
      impl_->observation.time, impl_->observation.state,
      optimized_state, optimized_input, planned_mode);
    impl_->observation.input = optimized_input;
    const auto wbc_start = std::chrono::steady_clock::now();
    wbc_solution = impl_->wbc->update(
      optimized_state, optimized_input, impl_->measured_rbd_state,
      planned_mode, period.seconds());
    impl_->timing.wbc_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - wbc_start).count();
    torque = wbc_solution.tail(static_cast<Eigen::Index>(kNumJoints));
  }
  auto position = ocs2::centroidal_model::getJointAngles(
    optimized_state, info);
  auto velocity = ocs2::centroidal_model::getJointVelocities(
    optimized_input, info);
  // The policy may predict the next contact mode before the measured policy
  // time reaches it.  Track both values so the physical transition itself is
  // also handed off smoothly; otherwise a second jump can occur when the
  // actual stance/swing mode catches up with the prediction.
  const bool mode_changed = policy_ready && (
    planned_mode != impl_->last_planned_mode || active_mode != impl_->last_active_mode);
  if (mode_changed && impl_->have_last_command) {
    impl_->handoff_start_time = input.timestamp;
    impl_->handoff_position = impl_->last_position_command;
    impl_->handoff_velocity = impl_->last_velocity_command;
    impl_->handoff_torque = impl_->last_torque_command;
    impl_->mode_handoff_active = true;
  }
  if (policy_ready) {
    impl_->last_active_mode = active_mode;
    impl_->last_planned_mode = planned_mode;
  }
  if (impl_->mode_handoff_active && impl_->handoff_position.size() == position.size() &&
    impl_->handoff_velocity.size() == velocity.size() &&
    impl_->handoff_torque.size() == torque.size() &&
    impl_->handoff_position.allFinite() && impl_->handoff_velocity.allFinite() &&
    impl_->handoff_torque.allFinite()) {
    const double duration = std::max(
      impl_->legged_interface->modelSettings().phaseTransitionStanceTime,
      std::max(period.seconds(), 1.0e-6));
    const double alpha = std::clamp(
      (input.timestamp - impl_->handoff_start_time) / duration, 0.0, 1.0);
    position = (1.0 - alpha) * impl_->handoff_position + alpha * position;
    velocity = (1.0 - alpha) * impl_->handoff_velocity + alpha * velocity;
    torque = (1.0 - alpha) * impl_->handoff_torque + alpha * torque;
    if (alpha >= 1.0) {impl_->mode_handoff_active = false;}
  }
  const bool command_finite = position.allFinite() && velocity.allFinite() && torque.allFinite();
  double max_position_jump = 0.0;
  double max_velocity_jump = 0.0;
  double max_torque_jump = 0.0;
  if (command_finite && impl_->have_last_command &&
    impl_->last_position_command.size() == position.size() &&
    impl_->last_velocity_command.size() == velocity.size() &&
    impl_->last_torque_command.size() == torque.size() &&
    impl_->last_position_command.allFinite() &&
    impl_->last_velocity_command.allFinite() &&
    impl_->last_torque_command.allFinite()) {
    max_position_jump = (position - impl_->last_position_command).cwiseAbs().maxCoeff();
    max_velocity_jump = (velocity - impl_->last_velocity_command).cwiseAbs().maxCoeff();
    max_torque_jump = (torque - impl_->last_torque_command).cwiseAbs().maxCoeff();
  }
  // Diagnostics only: these values are recorded and printed, never clamped
  // and never used to cancel a request or invalidate the output.
  impl_->last_position_command = position;
  impl_->last_velocity_command = velocity;
  impl_->last_torque_command = torque;
  impl_->have_last_command = true;

  output.backend = AlgorithmBackendId::OpenDog;
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
  estimate.acceleration_world.z() -= static_cast<float>(gravity_reference);
  estimate.gravity_magnitude = static_cast<float>(gravity_reference);
  estimate.gravity_valid = input.imu.gravity_valid;
  estimate.timestamp = input.timestamp;
  estimate.valid = true;
  ++impl_->command_sequence;
  for (std::size_t current_leg = 0; current_leg < kNumLegs; ++current_leg) {
    const std::size_t opendog_leg = impl_->opendog_leg_for_current[current_leg];
    auto & command = output.commands[current_leg];
    command.leg = static_cast<LegId>(current_leg);
    command.timestamp = static_cast<float>(input.timestamp);
    command.sequence = impl_->command_sequence;
    command.kp.setConstant(impl_->joint_kp);
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
  auto & timing = impl_->timing;
  timing.active_mode = active_mode;
  const auto now = std::chrono::steady_clock::now();
  timing.compute_ms = std::chrono::duration<double, std::milli>(now - compute_start).count();
  timing.mpc_ms = impl_->last_mpc_ms.load();
  timing.policy_age_ms = policy_ready ? 1000.0 *
    (impl_->observation.time - impl_->mrt->getCommand().mpcInitObservation_.time) : 0.0;
  timing.horizon_remaining_ms = policy_ready ? 1000.0 *
    (impl_->mrt->getPolicy().timeTrajectory_.back() - impl_->observation.time) : 0.0;
  timing.planned_mode = planned_mode;
  timing.command_finite = command_finite;
  timing.max_position_jump = max_position_jump;
  timing.max_velocity_jump = max_velocity_jump;
  timing.max_torque_jump = max_torque_jump;
  const bool wbc_warning = policy_ready && (
    impl_->wbc->lastSolveStatus() != 0 || impl_->wbc->lastSolutionStatus() != 0 ||
    !wbc_solution.allFinite());
  impl_->max_compute_ms = std::max(impl_->max_compute_ms, timing.compute_ms);
  if (now - impl_->last_timing_log >= std::chrono::seconds(1)) {
    if (mpc_warning) {
      std::printf(
        "[OpenDog][WARNING] MPC worker stopped; continuing with the current policy "
        "monitor_only=true\n");
    }
    if (wbc_warning) {
      std::printf(
        "[OpenDog][WARNING] WBC status=%d solution_status=%d finite=%s; "
        "continuing output monitor_only=true\n",
        impl_->wbc->lastSolveStatus(), impl_->wbc->lastSolutionStatus(),
        wbc_solution.allFinite() ? "true" : "false");
    }
    std::printf("[OpenDog][TIMING] compute_ms=%.3f max_compute_ms=%.3f wbc_ms=%.3f "
      "mpc_ms=%.3f policy_age_ms=%.3f horizon_remaining_ms=%.3f "
      "active_mode=%zu planned_mode=%zu command_finite=%s "
      "max_dpos=%.6f max_dvel=%.6f max_dtau=%.6f monitor_only=true\n",
      timing.compute_ms, impl_->max_compute_ms, timing.wbc_ms, timing.mpc_ms,
      timing.policy_age_ms, timing.horizon_remaining_ms, timing.active_mode,
      timing.planned_mode, timing.command_finite ? "true" : "false",
      timing.max_position_jump, timing.max_velocity_jump, timing.max_torque_jump);
    if (policy_ready) {
      std::printf("[OpenDog][WBC] solve_status=%d solution_status=%d mode=%zu finite=%s",
        impl_->wbc->lastSolveStatus(), impl_->wbc->lastSolutionStatus(), planned_mode,
        wbc_solution.allFinite() ? "true" : "false");
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const auto index = 3 * impl_->opendog_leg_for_current[leg];
        std::printf(" leg=%zu force_des=(%.3f,%.3f,%.3f) force_wbc=(%.3f,%.3f,%.3f)",
          leg, optimized_input(index), optimized_input(index + 1), optimized_input(index + 2),
          wbc_solution(info.generalizedCoordinatesNum + index),
          wbc_solution(info.generalizedCoordinatesNum + index + 1),
          wbc_solution(info.generalizedCoordinatesNum + index + 2));
      }
      std::printf(" monitor_only=true\n");
    }
    impl_->last_timing_log = now;
    impl_->max_compute_ms = 0.0;
  }
  return true;
#endif
}

void OpenDogAlgorithm::stop() noexcept
{
#if defined(MYMIT_ENABLE_OPENDOG)
  impl_->mpc_running.store(false);
  if (impl_->mpc_thread.joinable()) {impl_->mpc_thread.join();}
  if (impl_->mrt) {impl_->mrt->reset();}
  if (impl_->mpc) {impl_->mpc->reset();}
  if (impl_->reference) {impl_->reference->reset();}
  if (impl_->gait_command) {impl_->gait_command->reset();}
  impl_->initialized = false;
  impl_->initial_policy_ready.store(false);
  impl_->have_last_command = false;
  impl_->last_active_mode = static_cast<std::size_t>(
    ocs2::legged_robot::ModeNumber::STANCE);
  impl_->last_planned_mode = static_cast<std::size_t>(
    ocs2::legged_robot::ModeNumber::STANCE);
  impl_->mode_handoff_active = false;
#endif
}

void OpenDogAlgorithm::reset() noexcept
{
#if defined(MYMIT_ENABLE_OPENDOG)
  stop();
  impl_->time = 0.0;
  impl_->observation.state.setZero();
  impl_->observation.input.setZero();
#endif
}
