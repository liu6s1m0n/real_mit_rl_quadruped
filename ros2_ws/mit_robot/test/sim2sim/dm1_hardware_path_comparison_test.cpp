/**
 * @file dm1_hardware_path_comparison_test.cpp
 * @brief Compare MuJoCo-backed and hardware-feedback RobotRunner paths.
 * No CAN device, motor driver, or physical actuator is opened by this test.
 */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "RobotRunner.hpp"
#include "SimulationActuatorWriter.hpp"
#include "sensor/imu.hpp"
#include "sensor/leg.hpp"

namespace
{
using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
constexpr std::array<LegId, kNumLegs> kLegs{
  LegId::FR, LegId::FL, LegId::RR, LegId::RL};
constexpr double kRadiansToDegrees = 57.29577951308232;
constexpr std::array<const char *, kNumLegs> kLegNames{"FR", "FL", "RR", "RL"};
constexpr std::array<std::array<const char *, kJointsPerLeg>, kNumLegs> kJointNames{{
  {{"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"}},
  {{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}},
  {{"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}},
  {{"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}}
}};

ModelPointer loadModel()
{
  char error[1024]{};
  ModelPointer model(
    mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)),
    &mj_deleteModel);
  if (!model) {throw std::runtime_error(error);}
  return model;
}

DataPointer makeData(const mjModel * model)
{
  DataPointer data(mj_makeData(model), &mj_deleteData);
  if (!data) {throw std::runtime_error("cannot allocate MuJoCo data");}
  const int home = mj_name2id(model, mjOBJ_KEY, "home");
  if (home < 0) {throw std::runtime_error("DM1 home keyframe is missing");}
  mj_resetDataKeyframe(model, data.get(), home);
  mj_forward(model, data.get());
  return data;
}

struct PathReport
{
  double minimum_body_z = 1.0e9;
  double maximum_body_z = -1.0e9;
  double maximum_roll_error = 0.0;
  double maximum_pitch_error = 0.0;
  double maximum_height_error = 0.0;
  double peak_feedforward = 0.0;
  double peak_position_term = 0.0;
  double peak_velocity_term = 0.0;
  double peak_raw_torque = 0.0;
  double peak_first_100ms_torque = 0.0;
  std::array<double, kNumLegs> maximum_foot_lift{};
  std::size_t locomotion_frames = 0;
  std::size_t fallbacks = 0;
};

struct ComparisonReport
{
  PathReport simulation;
  PathReport hardware;
  double first_100ms_max_q_des_difference = 0.0;
  double first_100ms_max_torque_difference = 0.0;
  double first_100ms_max_estimated_rpy_difference = 0.0;
  bool stood = false;
  bool completed = false;
  std::string failure;
};

void recordPath(
  const mjData * data, const RobotRunner & runner,
  const std::array<int, kNumLegs> & foot_bodies,
  const std::array<double, kNumLegs> & stand_foot_z,
  const std::array<std::array<int, kJointsPerLeg>, kNumLegs> & qpos_addresses,
  const std::array<std::array<int, kJointsPerLeg>, kNumLegs> & qvel_addresses,
  int trunk, std::size_t step, FSM_StateName previous, PathReport & report)
{
  const double true_z = data->xpos[3 * trunk + 2];
  const mjtNum * rotation = data->xmat + 9 * trunk;
  const double true_roll = std::atan2(rotation[7], rotation[8]);
  const double true_pitch = std::atan2(-rotation[6],
    std::sqrt(rotation[0] * rotation[0] + rotation[3] * rotation[3]));
  const auto & estimate = runner.stateEstimate();
  report.minimum_body_z = std::min(report.minimum_body_z, true_z);
  report.maximum_body_z = std::max(report.maximum_body_z, true_z);
  report.maximum_roll_error = std::max(
    report.maximum_roll_error, std::abs(estimate.rpy.x() - true_roll));
  report.maximum_pitch_error = std::max(
    report.maximum_pitch_error, std::abs(estimate.rpy.y() - true_pitch));
  report.maximum_height_error = std::max(
    report.maximum_height_error,
    std::abs(estimate.position_world.z() - true_z));
  if (runner.currentStateName() == FSM_StateName::LOCOMOTION) {
    ++report.locomotion_frames;
  } else if (previous == FSM_StateName::LOCOMOTION &&
    runner.currentStateName() == FSM_StateName::BALANCE_STAND)
  {
    ++report.fallbacks;
  }

  const auto & commands = runner.jointCommands();
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    report.maximum_foot_lift[leg] = std::max(
      report.maximum_foot_lift[leg],
      data->xpos[3 * foot_bodies[leg] + 2] - stand_foot_z[leg]);
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto index = static_cast<Eigen::Index>(joint);
      const auto & command = commands[leg];
      const double p = command.kp[index] *
        (command.position_desired[index] - data->qpos[qpos_addresses[leg][joint]]);
      const double d = command.kd[index] *
        (command.velocity_desired[index] - data->qvel[qvel_addresses[leg][joint]]);
      const double ff = command.torque_feedforward[index];
      report.peak_feedforward = std::max(report.peak_feedforward, std::abs(ff));
      report.peak_position_term = std::max(report.peak_position_term, std::abs(p));
      report.peak_velocity_term = std::max(report.peak_velocity_term, std::abs(d));
      report.peak_raw_torque = std::max(report.peak_raw_torque, std::abs(ff + p + d));
      if (step < 50) {
        report.peak_first_100ms_torque = std::max(
          report.peak_first_100ms_torque, std::abs(ff + p + d));
      }
    }
  }
}

void printPath(const char * label, const PathReport & report)
{
  std::printf(
    "%s frames=%zu fallbacks=%zu body_z=[%.3f,%.3f] "
    "estimate_error=(roll %.2f deg,pitch %.2f deg,z %.1f mm) "
    "peak=(ff %.1f,P %.1f,D %.1f,total %.1f,first100ms %.1f) "
    "foot_lift=[%.3f %.3f %.3f %.3f] m\n",
    label, report.locomotion_frames, report.fallbacks,
    report.minimum_body_z, report.maximum_body_z,
    report.maximum_roll_error * kRadiansToDegrees,
    report.maximum_pitch_error * kRadiansToDegrees,
    report.maximum_height_error * 1000.0,
    report.peak_feedforward, report.peak_position_term,
    report.peak_velocity_term, report.peak_raw_torque,
    report.peak_first_100ms_torque,
    report.maximum_foot_lift[0], report.maximum_foot_lift[1],
    report.maximum_foot_lift[2], report.maximum_foot_lift[3]);
}

ComparisonReport runComparison(bool inject_attitude_bias)
{
  auto model = loadModel();
  auto sim_data = makeData(model.get());
  auto hardware_data = makeData(model.get());
  RobotRunner simulation(model.get(), sim_data.get(), RobotType::DM1);
  RobotRunner hardware(static_cast<float>(model->opt.timestep), RobotType::DM1);
  SimulationActuatorWriter sim_writer(model.get());
  SimulationActuatorWriter hardware_writer(model.get());
  SimImu hardware_imu(model.get(), hardware_data.get());
  std::array<std::unique_ptr<SimLeg>, kNumLegs> hardware_legs;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    hardware_legs[leg] = std::make_unique<SimLeg>(
      model.get(), hardware_data.get(), kLegs[leg]);
  }
  std::uint64_t imu_sequence = 0;
  const auto feedHardware = [&](bool bias) {
      auto imu = hardware_imu.read();
      imu.sequence = ++imu_sequence;
      if (bias) {
        constexpr float kDeg = 0.01745329251994329577F;
        const Eigen::Quaternionf attitude_offset =
          Eigen::AngleAxisf(4.0F * kDeg, Vec3<float>::UnitX()) *
          Eigen::AngleAxisf(-5.0F * kDeg, Vec3<float>::UnitY());
        imu.orientation_world_from_body =
          attitude_offset * imu.orientation_world_from_body;
        imu.orientation_world_from_body.normalize();
      }
      std::array<JointState<float>, kNumLegs> joints;
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        joints[leg] = hardware_legs[leg]->read();
      }
      return hardware.updateHardwareFeedback(
        imu, joints, static_cast<float>(hardware_data->time));
    };

  const int trunk = mj_name2id(model.get(), mjOBJ_BODY, "trunk");
  std::array<int, kNumLegs> foot_bodies{};
  std::array<std::array<int, kJointsPerLeg>, kNumLegs> qpos_addresses{};
  std::array<std::array<int, kJointsPerLeg>, kNumLegs> qvel_addresses{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_bodies[leg] = mj_name2id(model.get(), mjOBJ_BODY,
      (std::string(kLegNames[leg]) + "_foot").c_str());
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const int id = mj_name2id(model.get(), mjOBJ_JOINT, kJointNames[leg][joint]);
      qpos_addresses[leg][joint] = model->jnt_qposadr[id];
      qvel_addresses[leg][joint] = model->jnt_dofadr[id];
    }
  }

  ComparisonReport report;
  bool motion_started = false;
  bool sim_stand_pending = false;
  bool hardware_stand_pending = false;
  std::size_t settled_steps = 0;
  for (std::size_t step = 0; step < 20000; ++step) {
    if (!motion_started && sim_data->time >= 1.3) {
      simulation.prepareForMotionControl();
      hardware.prepareForMotionControl();
      motion_started = true;
      sim_stand_pending = true;
      hardware_stand_pending = true;
    }
    if (motion_started) {
      simulation.setStandingHeight(0.32F);
      hardware.setStandingHeight(0.32F);
      if (!feedHardware(false) || !simulation.run() || !hardware.run()) {
        report.failure = "feedback or controller rejected a standing frame";
        return report;
      }
      if (sim_stand_pending && simulation.requestStandUp()) {
        sim_stand_pending = false;
      }
      if (hardware_stand_pending && hardware.requestStandUp()) {
        hardware_stand_pending = false;
      }
    }
    sim_writer.write(simulation, sim_data.get(), motion_started);
    hardware_writer.write(hardware, hardware_data.get(), motion_started);
    mj_step(model.get(), sim_data.get());
    mj_step(model.get(), hardware_data.get());
    const bool both_standing = !sim_stand_pending && !hardware_stand_pending &&
      simulation.currentStateName() == FSM_StateName::BALANCE_STAND &&
      hardware.currentStateName() == FSM_StateName::BALANCE_STAND;
    const bool settled = both_standing &&
      std::abs(sim_data->xpos[3 * trunk + 2] - 0.32) < 0.02 &&
      std::abs(hardware_data->xpos[3 * trunk + 2] - 0.32) < 0.02 &&
      std::hypot(sim_data->qvel[0], sim_data->qvel[1]) < 0.05 &&
      std::hypot(hardware_data->qvel[0], hardware_data->qvel[1]) < 0.05;
    settled_steps = settled ? settled_steps + 1 : 0;
    if (settled_steps >= 500) {
      report.stood = true;
      break;
    }
  }
  if (!report.stood) {
    report.failure = "both paths did not reach settled BalanceStand";
    return report;
  }

  std::array<double, kNumLegs> sim_stand_foot_z{};
  std::array<double, kNumLegs> hardware_stand_foot_z{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    sim_stand_foot_z[leg] = sim_data->xpos[3 * foot_bodies[leg] + 2];
    hardware_stand_foot_z[leg] = hardware_data->xpos[3 * foot_bodies[leg] + 2];
  }

  auto sim_previous = simulation.currentStateName();
  auto hardware_previous = hardware.currentStateName();
  for (std::size_t step = 0; step < 2000; ++step) {
    simulation.setLocomotionGait(GaitType::TROT);
    hardware.setLocomotionGait(GaitType::TROT);
    simulation.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
    hardware.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
    simulation.setControlMode(ControlMode::Locomotion);
    hardware.setControlMode(ControlMode::Locomotion);
    if (!feedHardware(inject_attitude_bias) || !simulation.run() || !hardware.run()) {
      report.failure = "feedback or controller rejected a locomotion frame";
      return report;
    }
    recordPath(sim_data.get(), simulation, foot_bodies,
      sim_stand_foot_z, qpos_addresses, qvel_addresses, trunk, step,
      sim_previous, report.simulation);
    recordPath(hardware_data.get(), hardware, foot_bodies,
      hardware_stand_foot_z, qpos_addresses, qvel_addresses, trunk, step,
      hardware_previous, report.hardware);
    sim_previous = simulation.currentStateName();
    hardware_previous = hardware.currentStateName();

    if (step < 50) {
      report.first_100ms_max_estimated_rpy_difference = std::max(
        report.first_100ms_max_estimated_rpy_difference,
        static_cast<double>((simulation.stateEstimate().rpy -
        hardware.stateEstimate().rpy).cwiseAbs().maxCoeff()));
      const auto & sim_commands = simulation.jointCommands();
      const auto & hardware_commands = hardware.jointCommands();
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
          const auto index = static_cast<Eigen::Index>(joint);
          report.first_100ms_max_q_des_difference = std::max(
            report.first_100ms_max_q_des_difference,
            static_cast<double>(std::abs(
              sim_commands[leg].position_desired[index] -
              hardware_commands[leg].position_desired[index])));
          const auto rawTorque = [&](const JointCommand<float> & command,
            const mjData * data) {
              return command.torque_feedforward[index] +
                command.kp[index] * (command.position_desired[index] -
                data->qpos[qpos_addresses[leg][joint]]) +
                command.kd[index] * (command.velocity_desired[index] -
                data->qvel[qvel_addresses[leg][joint]]);
            };
          report.first_100ms_max_torque_difference = std::max(
            report.first_100ms_max_torque_difference,
            static_cast<double>(std::abs(rawTorque(sim_commands[leg], sim_data.get()) -
              rawTorque(hardware_commands[leg], hardware_data.get()))));
        }
      }
    }
    sim_writer.write(simulation, sim_data.get(), true);
    hardware_writer.write(hardware, hardware_data.get(), true);
    mj_step(model.get(), sim_data.get());
    mj_step(model.get(), hardware_data.get());
  }
  report.completed = true;
  return report;
}

void printComparison(const char * scenario, const ComparisonReport & report)
{
  std::printf("comparison %s first100ms_delta=(q_des %.4f rad,torque %.2f Nm,"
    "estimated_rpy %.2f deg)\n", scenario,
    report.first_100ms_max_q_des_difference,
    report.first_100ms_max_torque_difference,
    report.first_100ms_max_estimated_rpy_difference * kRadiansToDegrees);
  printPath("  simulation", report.simulation);
  printPath("  hardware  ", report.hardware);
}

TEST(Dm1HardwarePathComparison, MatchedMuJoCoSensors)
{
  const auto report = runComparison(false);
  ASSERT_TRUE(report.stood) << report.failure;
  printComparison("matched", report);
  ASSERT_TRUE(report.completed) << report.failure;
  EXPECT_EQ(report.simulation.fallbacks, 0U);
  EXPECT_EQ(report.hardware.fallbacks, 0U);
  EXPECT_EQ(report.simulation.locomotion_frames, 2000U);
  EXPECT_EQ(report.hardware.locomotion_frames, 2000U);
  EXPECT_GT(report.simulation.minimum_body_z, 0.29);
  EXPECT_GT(report.hardware.minimum_body_z, 0.29);
  EXPECT_LT(report.simulation.peak_raw_torque, 30.0);
  EXPECT_LT(report.hardware.peak_raw_torque, 30.0);
  EXPECT_LT(report.hardware.maximum_roll_error * kRadiansToDegrees, 1.5);
  EXPECT_LT(report.hardware.maximum_pitch_error * kRadiansToDegrees, 1.5);
  EXPECT_LT(report.hardware.maximum_height_error, 0.015);
  EXPECT_LT(report.first_100ms_max_q_des_difference, 0.05);
  EXPECT_LT(report.first_100ms_max_torque_difference, 1.0);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_GT(report.simulation.maximum_foot_lift[leg], 0.015) << leg;
    EXPECT_GT(report.hardware.maximum_foot_lift[leg], 0.015) << leg;
  }
}

TEST(Dm1HardwarePathComparison, AttitudeOffsetSensitivity)
{
  // Diagnostic stress only: this is not a replay of measured hardware IMU data.
  const auto report = runComparison(true);
  ASSERT_TRUE(report.stood) << report.failure;
  printComparison("imu_orientation_plus4_minus5deg", report);
  ASSERT_TRUE(report.completed) << report.failure;
  EXPECT_GT(report.first_100ms_max_estimated_rpy_difference, 0.01);
}
}  // namespace
