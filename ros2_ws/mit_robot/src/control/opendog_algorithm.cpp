#include "control/opendog_algorithm.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
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
#if defined(MYMIT_ENABLE_OPENDOG)
  std::shared_ptr<rclcpp::Node> node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread mpc_thread;
  std::atomic<bool> mpc_running{false};
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
#endif
};

OpenDogAlgorithm::OpenDogAlgorithm(std::string config_directory, double control_period)
: impl_(std::make_unique<Impl>(std::move(config_directory), control_period))
{}

OpenDogAlgorithm::~OpenDogAlgorithm() {stop();}

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
  if (impl_->initialized && !impl_->mpc_running.load() && impl_->mpc_error) {
    std::rethrow_exception(impl_->mpc_error);
  }

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
  const auto mode = impl_->initialized ?
    impl_->mrt->getPolicy().modeSchedule_.modeAtTime(impl_->time) :
    ocs2::legged_robot::ModeNumber::STANCE;
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
  const auto yaw_last = impl_->observation.state(9);
  impl_->observation.state = impl_->rbd_conversions->computeCentroidalStateFromRbdModel(
    impl_->measured_rbd_state);
  const auto yaw_delta = impl_->observation.state(9) - yaw_last;
  impl_->observation.state(9) = yaw_last + std::atan2(std::sin(yaw_delta), std::cos(yaw_delta));
  impl_->observation.mode = impl_->state_estimate->getMode();

  // The shared desired-state snapshot is the command boundary for both
  // algorithms.  Keep the OpenDog reference manager current without routing
  // commands through its original controller_manager hardware plugin.
  impl_->mrt->setCurrentObservation(impl_->observation);
  impl_->mrt->getReferenceManager().setTargetTrajectories(
    impl_->reference->update(input.desired, impl_->observation));
  impl_->gait_command->setMode(input.desired.mode);

  if (!impl_->initialized) {
    impl_->observation.input.setZero();
    while (!impl_->mrt->initialPolicyReceived()) {
      impl_->mrt->advanceMpc();
    }
    impl_->initialized = true;
    impl_->mpc_error = nullptr;
    impl_->mpc_running.store(true);
    impl_->mpc_thread = std::thread([this]() {
        try {
          while (impl_->mpc_running.load()) {
            ocs2::executeAndSleep([this]() {
                impl_->executor->spin_some();
                impl_->mrt->advanceMpc();
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
  impl_->mrt->updatePolicy();

  ocs2::vector_t optimized_state;
  ocs2::vector_t optimized_input;
  std::size_t planned_mode = 0;
  impl_->mrt->evaluatePolicy(
    impl_->observation.time, impl_->observation.state,
    optimized_state, optimized_input, planned_mode);
  impl_->observation.input = optimized_input;
  const auto wbc_solution = impl_->wbc->update(
    optimized_state, optimized_input, impl_->measured_rbd_state,
    planned_mode, period.seconds());
  const auto torque = wbc_solution.tail(static_cast<Eigen::Index>(kNumJoints));
  const auto position = ocs2::centroidal_model::getJointAngles(
    optimized_state, info);
  const auto velocity = ocs2::centroidal_model::getJointVelocities(
    optimized_input, info);

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
  estimate.acceleration_world.z() -= 9.81F;
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
  impl_->initialized = false;
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
