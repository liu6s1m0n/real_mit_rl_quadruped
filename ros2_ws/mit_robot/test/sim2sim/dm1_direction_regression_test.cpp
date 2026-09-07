#include <array>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "RobotRunner.hpp"
#include "SimulationActuatorWriter.hpp"
#include "SimulationDiagnostics.hpp"
#include "controller/frozen_dwaq_policy.hpp"

namespace
{
using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

constexpr float kRlHeight = 0.38F;
constexpr float kGenericStandHeight = 0.39F;
constexpr double kIsaacPhysicsTimestep = 0.002;
constexpr std::size_t kStartupStepLimit = 6000;  // startup budget; entry exits earlier.
// RL entry may spend several seconds moving from the generic 0.39 m stand
// target to the 0.38 m training target at the shared height-rate limit. The
// acceptance clock starts only once LOCOMOTION is active, so leave that
// transition outside the ten-second measurement horizon.
constexpr std::size_t kDirectionStepLimit = 12000;  // direction budget; 10 s normally exits.

struct DirectionCase
{
  const char * name;
  Vec3<float> command;
};

// Isaac model_4210 flat-eval ratios, taken from
// /home/simon/RL_Robot/logs/trot/dm1_trot_directional_hip_refine_4160/
// Sep04_17-39-42_directional_hip_from4160_50/evaluation_model_4210_gait.csv.
// The regression compares the scaled command response to this capability band,
// rather than accepting a mere sign flip. The band is deliberately broad
// enough to cover the different command magnitudes used by this harness.
constexpr float kIsaacCapabilityRatioMin = 0.35F;
constexpr float kIsaacCapabilityRatioMax = 1.50F;
constexpr float kIsaacForwardRatio = 0.2946269F / 0.36F;
constexpr float kIsaacBackwardRatio = 0.2893912F / 0.36F;
constexpr float kIsaacLeftRatio = 0.1666428F / 0.25F;
constexpr float kIsaacRightRatio = 0.1202483F / 0.25F;
constexpr float kIsaacLeftLongitudinalCrosstalkRatio = 0.0021824F / 0.1666428F;
constexpr float kIsaacRightLongitudinalCrosstalkRatio = 0.0138787F / 0.1202483F;
constexpr float kCrosstalkMargin = 4.0F;

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
  if (!data) {throw std::runtime_error("unable to allocate MuJoCo test data");}
  const int home = mj_name2id(model, mjOBJ_KEY, "home");
  if (home < 0) {throw std::runtime_error("DM1 scene has no home keyframe");}
  mj_resetDataKeyframe(model, data.get(), home);
  mj_forward(model, data.get());
  return data;
}

struct FootTraceSnapshot
{
  std::array<int, kNumLegs> contact{};
  std::array<float, kNumLegs> force{};
  std::array<float, kNumLegs> slip{};
};

FootTraceSnapshot readFootTrace(const mjModel * model, const mjData * data)
{
  constexpr std::array<const char *, kNumLegs> kFootNames{"FR", "FL", "RR", "RL"};
  FootTraceSnapshot snapshot{};
  const int floor = mj_name2id(model, mjOBJ_GEOM, "floor");
  std::array<int, kNumLegs> foot_geoms{};
  std::array<int, kNumLegs> foot_bodies{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_geoms[leg] = mj_name2id(model, mjOBJ_GEOM, kFootNames[leg]);
    const std::string body_name = std::string(kFootNames[leg]) + "_foot";
    foot_bodies[leg] = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (int index = 0; index < data->ncon; ++index) {
      const auto & contact = data->contact[index];
      if (!((contact.geom1 == floor && contact.geom2 == foot_geoms[leg]) ||
        (contact.geom2 == floor && contact.geom1 == foot_geoms[leg])))
      {
        continue;
      }
      snapshot.contact[leg] = 1;
      mjtNum contact_force[6]{};
      mj_contactForce(model, data, index, contact_force);
      snapshot.force[leg] += static_cast<float>(std::sqrt(
        contact_force[0] * contact_force[0] +
        contact_force[1] * contact_force[1] +
        contact_force[2] * contact_force[2]));
    }
    mjtNum velocity[6]{};
    mj_objectVelocity(model, data, mjOBJ_BODY, foot_bodies[leg], velocity, 0);
    snapshot.slip[leg] = static_cast<float>(std::sqrt(
      velocity[3] * velocity[3] + velocity[4] * velocity[4]));
  }
  return snapshot;
}

bool genericStandStable(const mjModel * model, const mjData * data)
{
  const int trunk = mj_name2id(model, mjOBJ_BODY, "trunk");
  if (trunk < 0 || std::abs(data->xpos[3 * trunk + 2] - 0.39) > 0.015) {
    return false;
  }
  for (int dof = 6; dof < model->nv; ++dof) {
    if (std::abs(data->qvel[dof]) > 0.25) {return false;}
  }
  const mjtNum * rotation = data->xmat + 9 * trunk;
  const double roll = std::atan2(rotation[7], rotation[8]);
  const double pitch = std::atan2(
    -rotation[6], std::sqrt(rotation[0] * rotation[0] + rotation[3] * rotation[3]));
  return std::abs(roll) <= 0.08 && std::abs(pitch) <= 0.08;
}

bool step(
  const mjModel * model, mjData * data, RobotRunner & runner,
  SimulationActuatorWriter & actuator_writer, SimulationDiagnostics & diagnostics,
  float standing_height, const Vec3<float> & command, bool direction_active,
  bool run_control = true)
{
  runner.setStandingHeight(standing_height);
  const bool control_valid = run_control ? runner.run() : true;
  std::array<std::size_t, kNumLegs> saturation_by_leg{};
  actuator_writer.write(runner, data, saturation_by_leg);
  const std::size_t target_joint_limit_hits =
    countSimulationTargetJointLimitHits(runner);
  diagnostics.observe(
    data, runner.stateEstimate(), control_valid, direction_active, command,
    runner.hasRlRawAction() ? &runner.rlLastRawAction() : nullptr,
    target_joint_limit_hits,
    &saturation_by_leg);
  mj_step(model, data);
  return control_valid;
}

SimulationDiagnosticReport runDirection(const DirectionCase & direction)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  if (std::abs(model->opt.timestep - kIsaacPhysicsTimestep) > 1.0e-9) {
    throw std::runtime_error(
            "DM1 Sim2Sim requires the Isaac 0.002 s MuJoCo integration timestep");
  }
  // Keep the validated MuJoCo integration step; callers can select a separate
  // controller rate for timing experiments without changing the policy.
  double control_period = model->opt.timestep;
  if (const char * control_hz = std::getenv("DM1_SIM2SIM_CONTROL_HZ")) {
    const double requested_hz = std::atof(control_hz);
    if (std::isfinite(requested_hz) && requested_hz >= 100.0 &&
      requested_hz <= 1000.0)
    {
      control_period = 1.0 / requested_hz;
    }
  }
  RobotRunner runner(
    model.get(), data.get(), RobotType::DM1, static_cast<float>(control_period));
  runner.setRlPolicy(std::make_shared<FrozenDwaqPolicy>());
  SimulationActuatorWriter actuator_writer(model.get());
  SimulationDiagnostics diagnostics(model.get());
  std::ofstream trace;
  std::uint64_t last_trace_sequence = 0;
  if (std::string(direction.name) == "left") {
    const char * trace_path = std::getenv("DM1_SIM2SIM_TRACE_PATH");
    trace.open(trace_path != nullptr ? trace_path :
      "/tmp/dm1_sim2sim_left_policy_trace.csv");
    if (trace) {
      trace << "sequence,time_s";
      for (std::size_t i = 0; i < kRlObservationSize; ++i) {trace << ",obs_" << i;}
      for (std::size_t i = 0; i < kRlObservationSize * kRlHistoryLength; ++i) {
        trace << ",history_" << i;
      }
      for (std::size_t i = 0; i < kRlActionSize; ++i) {trace << ",raw_" << i;}
      for (std::size_t i = 0; i < kRlActionSize; ++i) {trace << ",filtered_" << i;}
      for (std::size_t i = 0; i < kRlActionSize; ++i) {trace << ",target_" << i;}
      for (std::size_t i = 0; i < kNumLegs; ++i) {trace << ",contact_force_" << i;}
      for (std::size_t i = 0; i < kNumLegs; ++i) {trace << ",foot_slip_" << i;}
      for (std::size_t i = 0; i < kNumLegs; ++i) {trace << ",contact_" << i;}
      trace << '\n';
    }
  }

  // Reproduce the real entry path: initialization -> StandUp -> BalanceStand.
  bool requested_stand_up = false;
  bool reached_balance_stand = false;
  double next_control_time = 0.0;
  for (std::size_t step_index = 0; step_index < kStartupStepLimit; ++step_index) {
    if (!requested_stand_up && data->time >= 1.3) {
      requested_stand_up = runner.requestStandUp();
    }
    const bool run_control = data->time + 0.5 * model->opt.timestep >= next_control_time;
    if (run_control) {next_control_time += control_period;}
    step(
      model.get(), data.get(), runner, actuator_writer, diagnostics,
      kGenericStandHeight, Vec3<float>::Zero(), false, run_control);
    if (runner.currentStateName() == FSM_StateName::BALANCE_STAND) {
      reached_balance_stand = true;
      if (data->time >= 2.5 && genericStandStable(model.get(), data.get())) {break;}
    }
  }
  if (!reached_balance_stand) {
    throw std::runtime_error(
            std::string("did not reach BalanceStand before direction ") + direction.name);
  }

  diagnostics.reset();
  runner.setLocomotionVelocityCommand(
    direction.command.x(), direction.command.y(), direction.command.z());
  if (direction.command.isZero(0.0F)) {
    runner.setControlMode(ControlMode::BalanceStand);
  } else {
    runner.setControlMode(ControlMode::WalkRl);
  }

  std::size_t active_steps = 0;
  bool measurement_started = false;
  double active_start_time = -1.0;
  for (std::size_t step_index = 0; step_index < kDirectionStepLimit; ++step_index) {
    const bool active = runner.currentStateName() == FSM_StateName::LOCOMOTION &&
      !direction.command.isZero(0.0F);
    // Do not charge RL posture-transition frames as a direction failure. The
    // policy clock and all diagnostics start on the first actual LOCOMOTION
    // frame, after RobotRunner's entry contract has become true.
    if (active && !measurement_started) {
      EXPECT_TRUE(runner.rlEntryReady()) << direction.name;
      diagnostics.reset();
      measurement_started = true;
      active_start_time = data->time;
    }
    const bool run_control = data->time + 0.5 * model->opt.timestep >= next_control_time;
    if (run_control) {next_control_time += control_period;}
    step(
      model.get(), data.get(), runner, actuator_writer, diagnostics,
      kRlHeight, direction.command, active, run_control);
    if (trace && runner.hasRlFrameTrace() &&
      runner.rlLastFrameTrace().sequence != last_trace_sequence &&
      runner.rlLastFrameTrace().sequence <= 100)
    {
      const auto & frame = runner.rlLastFrameTrace();
      trace << frame.sequence << ',' <<
        (active_start_time >= 0.0 ? data->time - active_start_time : data->time);
      for (const float value : frame.observation) {trace << ',' << value;}
      for (const float value : frame.history) {trace << ',' << value;}
      for (const float value : frame.raw_action) {trace << ',' << value;}
      for (const float value : frame.filtered_action) {trace << ',' << value;}
      for (const float value : frame.target_position) {trace << ',' << value;}
      const auto foot_trace = readFootTrace(model.get(), data.get());
      for (const float value : foot_trace.force) {trace << ',' << value;}
      for (const float value : foot_trace.slip) {trace << ',' << value;}
      for (const int value : foot_trace.contact) {trace << ',' << value;}
      trace << '\n';
      last_trace_sequence = frame.sequence;
    }
    active_steps += active ? 1U : 0U;
    if (!direction.command.isZero(0.0F) && measurement_started &&
      data->time - active_start_time >= 10.0)
    {
      break;
    }
  }
  return diagnostics.report();
}

void expectDirectionMetrics(
  const DirectionCase & direction, const SimulationDiagnosticReport & report)
{
  const auto & window_1_2 = report.direction_windows[1];
  const auto & window_2_5 = report.direction_windows[2];
  const auto & window_5_10 = report.direction_windows[3];
  ASSERT_GT(window_5_10.samples, 1000U) << direction.name;
  const auto actual_1_2 = window_1_2.meanActual();
  const auto actual_2_5 = window_2_5.meanActual();
  const auto actual_5_10 = window_5_10.meanActual();
  std::cout << "sim2sim " << direction.name
            << " mean_1_2_body=(" << actual_1_2[0] << ", " << actual_1_2[1]
            << ", " << actual_1_2[2] << ") mean_2_5_body=("
            << actual_2_5[0] << ", " << actual_2_5[1] << ", " << actual_2_5[2]
            << ") mean_5_10_body=(" << actual_5_10[0] << ", "
            << actual_5_10[1] << ", " << actual_5_10[2] << ") fall_time="
            << report.fall_time_s << " calf_frames=" << report.calf_collision_frames
            << " observed=" << report.observed_frames
            << " calf_ratio=" << report.calfContactRatio()
            << " raw_max=" << report.raw_action_max_abs
            << " raw_over1=" << report.rawActionOverOneRatio()
            << " joint_limits=" << report.target_joint_limit_hits
            << " torque_speed_sat=" << report.torque_speed_saturation_count
            << " torque_speed_sat_by_leg=(" << report.torque_speed_saturation_by_leg[0]
            << "," << report.torque_speed_saturation_by_leg[1]
            << "," << report.torque_speed_saturation_by_leg[2]
            << "," << report.torque_speed_saturation_by_leg[3] << ")"
            << " foot_slip_mean=" << report.meanFootSlipSpeed()
            << " net_body=(" << report.net_displacement_body_x << ","
            << report.net_displacement_body_y << ")"
            << " foot_contact_force_mean_by_leg=("
            << report.foot_contact_by_leg[0].meanContactForce() << ","
            << report.foot_contact_by_leg[1].meanContactForce() << ","
            << report.foot_contact_by_leg[2].meanContactForce() << ","
            << report.foot_contact_by_leg[3].meanContactForce() << ")"
            << " foot_slip_by_leg=("
            << report.foot_contact_by_leg[0].meanSlipSpeed() << ","
            << report.foot_contact_by_leg[1].meanSlipSpeed() << ","
            << report.foot_contact_by_leg[2].meanSlipSpeed() << ","
            << report.foot_contact_by_leg[3].meanSlipSpeed() << ")"
            << " contact_transitions_by_leg=("
            << report.foot_contact_by_leg[0].contact_transitions << ","
            << report.foot_contact_by_leg[1].contact_transitions << ","
            << report.foot_contact_by_leg[2].contact_transitions << ","
            << report.foot_contact_by_leg[3].contact_transitions << ")\n";
  EXPECT_LT(report.fall_time_s, 0.25) << direction.name;
  EXPECT_EQ(report.calf_collision_frames, 0U) << direction.name;
  EXPECT_EQ(report.target_joint_limit_hits, 0U) << direction.name;
  EXPECT_EQ(report.torque_speed_saturation_count, 0U) << direction.name;
  EXPECT_GT(report.minimum_height, 0.30F) << direction.name;
  EXPECT_LT(report.maximum_absolute_pitch, 0.35F) << direction.name;
  EXPECT_LT(report.meanFootSlipSpeed(), 0.15F) << direction.name;
  if (direction.command.y() > 0.0F) {
    const float expected = direction.command.y() * kIsaacLeftRatio;
    EXPECT_GT(actual_5_10[1], expected * kIsaacCapabilityRatioMin) << direction.name;
    EXPECT_LT(actual_5_10[1], expected * kIsaacCapabilityRatioMax) << direction.name;
    const float measured_ratio = std::abs(actual_5_10[0]) /
      std::max(std::abs(actual_5_10[1]), 1.0e-3F);
    EXPECT_LE(
      measured_ratio, kCrosstalkMargin * kIsaacLeftLongitudinalCrosstalkRatio)
      << direction.name << " measured vx/vy=" << measured_ratio;
    EXPECT_GT(report.net_displacement_body_y, 0.0F) << direction.name;
    EXPECT_GT(
      std::abs(report.net_displacement_body_y),
      3.0F * std::abs(report.net_displacement_body_x)) << direction.name;
  } else if (direction.command.y() < 0.0F) {
    const float expected = direction.command.y() * kIsaacRightRatio;
    EXPECT_LT(actual_5_10[1], expected * kIsaacCapabilityRatioMin) << direction.name;
    EXPECT_GT(actual_5_10[1], expected * kIsaacCapabilityRatioMax) << direction.name;
    const float measured_ratio = std::abs(actual_5_10[0]) /
      std::max(std::abs(actual_5_10[1]), 1.0e-3F);
    EXPECT_LE(
      measured_ratio, kCrosstalkMargin * kIsaacRightLongitudinalCrosstalkRatio)
      << direction.name << " measured vx/vy=" << measured_ratio;
    EXPECT_LT(report.net_displacement_body_y, 0.0F) << direction.name;
    EXPECT_GT(
      std::abs(report.net_displacement_body_y),
      3.0F * std::abs(report.net_displacement_body_x)) << direction.name;
  } else if (direction.command.x() > 0.0F) {
    const float expected = direction.command.x() * kIsaacForwardRatio;
    EXPECT_GT(actual_5_10[0], expected * kIsaacCapabilityRatioMin) << direction.name;
    EXPECT_LT(actual_5_10[0], expected * kIsaacCapabilityRatioMax) << direction.name;
  } else if (direction.command.x() < 0.0F) {
    const float expected = direction.command.x() * kIsaacBackwardRatio;
    EXPECT_LT(actual_5_10[0], expected * kIsaacCapabilityRatioMin) << direction.name;
    EXPECT_GT(actual_5_10[0], expected * kIsaacCapabilityRatioMax) << direction.name;
  } else if (direction.command.z() > 0.0F) {
    EXPECT_GT(actual_5_10[2], 0.03F) << direction.name;
  } else if (direction.command.z() < 0.0F) {
    EXPECT_LT(actual_5_10[2], -0.03F) << direction.name;
  }
}
}  // namespace

TEST(Dm1Sim2Sim, DirectionalTenSecondRegression)
{
  const std::array<DirectionCase, 7> cases{{
      {"stand", Vec3<float>::Zero()},
      {"forward", Vec3<float>(0.18F, 0.0F, 0.0F)},
      {"backward", Vec3<float>(-0.30F, 0.0F, 0.0F)},
      {"left", Vec3<float>(0.0F, 0.20F, 0.0F)},
      {"right", Vec3<float>(0.0F, -0.20F, 0.0F)},
      {"rotate_ccw", Vec3<float>(0.0F, 0.0F, 0.30F)},
      {"rotate_cw", Vec3<float>(0.0F, 0.0F, -0.30F)}}};
  const char * only_direction = std::getenv("DM1_SIM2SIM_ONLY_DIRECTION");
  for (const auto & direction : cases) {
    if (only_direction != nullptr && std::string(only_direction) != direction.name) {
      continue;
    }
    const auto report = runDirection(direction);
    if (direction.command.isZero(0.0F)) {
      std::cout << "sim2sim stand fall_time=" << report.fall_time_s
                << " calf_frames=" << report.calf_collision_frames
                << " observed=" << report.observed_frames
                << " calf_ratio=" << report.calfContactRatio() << '\n';
      EXPECT_LT(report.fall_time_s, 0.25) << direction.name;
      EXPECT_EQ(report.calf_collision_frames, 0U) << direction.name;
      EXPECT_EQ(report.target_joint_limit_hits, 0U) << direction.name;
      EXPECT_EQ(report.torque_speed_saturation_count, 0U) << direction.name;
      EXPECT_GT(report.minimum_height, 0.30F) << direction.name;
      EXPECT_LT(report.maximum_absolute_pitch, 0.35F) << direction.name;
      EXPECT_LT(report.meanFootSlipSpeed(), 0.15F) << direction.name;
    } else {
      expectDirectionMetrics(direction, report);
    }
  }
}

TEST(Dm1Sim2Sim, RlEntryCannotBypassWhenHeightAlreadyMatches)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  runner.setRlPolicy(std::make_shared<FrozenDwaqPolicy>());
  SimulationActuatorWriter actuator_writer(model.get());
  SimulationDiagnostics diagnostics(model.get());

  bool requested_stand_up = false;
  bool reached_balance_stand = false;
  for (std::size_t step_index = 0; step_index < kStartupStepLimit; ++step_index) {
    if (!requested_stand_up && data->time >= 1.3) {
      requested_stand_up = runner.requestStandUp();
    }
    step(
      model.get(), data.get(), runner, actuator_writer, diagnostics,
      kGenericStandHeight, Vec3<float>::Zero(), false);
    if (runner.currentStateName() == FSM_StateName::BALANCE_STAND &&
      data->time >= 2.5 && genericStandStable(model.get(), data.get()))
    {
      reached_balance_stand = true;
      break;
    }
  }
  ASSERT_TRUE(reached_balance_stand);

  // Force only the measured height to the RL target. The remaining posture
  // contract must still be latched for 0.20 s before LOCOMOTION is allowed.
  data->qpos[2] = kRlHeight;
  mj_forward(model.get(), data.get());
  runner.setLocomotionVelocityCommand(0.18F, 0.0F, 0.0F);
  runner.setControlMode(ControlMode::WalkRl);
  for (std::size_t index = 0; index < 10; ++index) {
    step(
      model.get(), data.get(), runner, actuator_writer, diagnostics,
      kRlHeight, Vec3<float>(0.18F, 0.0F, 0.0F), false);
  }
  EXPECT_FALSE(runner.rlEntryReady());
}
