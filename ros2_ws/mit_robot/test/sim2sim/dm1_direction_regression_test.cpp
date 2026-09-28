#include <array>
#include <algorithm>
#include <cmath>
#include <cstdlib>
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
#include "teleop/operator_command.hpp"

namespace
{
using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

constexpr float kRlHeight = 0.38F;
constexpr float kGenericStandHeight = 0.32F;
constexpr double kIsaacPhysicsTimestep = 0.002;
constexpr std::size_t kStartupStepLimit = 6000;  // startup budget; entry exits earlier.
// RL entry may spend several seconds moving from the generic 0.32 m stand
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
  std::array<float, kNumLegs> normal_force{};
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
      snapshot.normal_force[leg] += static_cast<float>(std::abs(contact_force[0]));
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
  if (trunk < 0 || std::abs(data->xpos[3 * trunk + 2] - 0.32) > 0.015) {
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
  actuator_writer.write(runner, data);
  const std::size_t target_joint_limit_hits =
    countSimulationTargetJointLimitHits(runner);
  diagnostics.observe(
    data, runner.stateEstimate(), control_valid, direction_active, command,
    target_joint_limit_hits);
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
            << " joint_limits=" << report.target_joint_limit_hits
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
                << " calf_ratio=" << report.calfContactRatio()
                << " foot_contact_samples=("
                << report.foot_contact_by_leg[0].contact_samples << ","
                << report.foot_contact_by_leg[1].contact_samples << ","
                << report.foot_contact_by_leg[2].contact_samples << ","
                << report.foot_contact_by_leg[3].contact_samples << ")\n";
      EXPECT_LT(report.fall_time_s, 0.25) << direction.name;
      EXPECT_EQ(report.calf_collision_frames, 0U) << direction.name;
      EXPECT_EQ(report.target_joint_limit_hits, 0U) << direction.name;
      EXPECT_GT(report.minimum_height, 0.30F) << direction.name;
      EXPECT_LT(report.maximum_absolute_pitch, 0.35F) << direction.name;
      EXPECT_LT(report.meanFootSlipSpeed(), 0.15F) << direction.name;
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        EXPECT_GT(
          report.foot_contact_by_leg[leg].contact_samples,
          report.observed_frames * 95U / 100U) << direction.name << " leg=" << leg;
      }
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

  // The production loop reapplies the selected mode while motion is latched.
  // Repeating the same RL request must preserve the entry stability clock and
  // allow the posture contract to complete.
  for (std::size_t index = 0; index < 1000; ++index) {
    runner.setControlMode(ControlMode::WalkRl);
    ASSERT_TRUE(step(
        model.get(), data.get(), runner, actuator_writer, diagnostics,
        kRlHeight, Vec3<float>(0.18F, 0.0F, 0.0F), false));
  }
  EXPECT_TRUE(runner.rlEntryReady());
}

TEST(Dm1Sim2Sim, ProneDownUsesMpcWhenRlWasSelected)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter actuator_writer(model.get());
  SimulationDiagnostics diagnostics(model.get());
  std::size_t rl_policy_calls = 0;
  runner.setRlPolicy(std::make_shared<CallbackRlPolicy>(
      [&rl_policy_calls](
        const std::array<float, kRlObservationSize> &,
        const std::array<float, kRlObservationSize * kRlHistoryLength> &,
        std::array<float, kRlActionSize> & action) {
        ++rl_policy_calls;
        action.fill(0.0F);
        return true;
      },
      RlPolicyMetadata{
        "model_4210", kDm1FlatCheckpointSha256, false, true, true}));

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

  // Select RL as the walking mode first, then request prone-down. The prone
  // transition must replace that selection with the existing MPC path.
  runner.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
  runner.setControlMode(ControlMode::WalkRl);
  for (std::size_t step_index = 0; step_index < 3000; ++step_index) {
    ASSERT_TRUE(step(
      model.get(), data.get(), runner, actuator_writer, diagnostics,
      kRlHeight, Vec3<float>::Zero(), false));
    if (runner.rlEntryReady()) {break;}
  }
  ASSERT_TRUE(runner.rlEntryReady());
  // rlEntryReady() becomes true on the FSM handoff frame; execute one actual
  // locomotion cycle so the injected policy is observed before the prone
  // request is issued.
  ASSERT_TRUE(step(
      model.get(), data.get(), runner, actuator_writer, diagnostics,
      kRlHeight, Vec3<float>::Zero(), false));
  ASSERT_GT(rl_policy_calls, 0U);

  ASSERT_TRUE(runner.requestProneDown());
  bool entered_prone_mpc = false;
  std::size_t rl_calls_at_prone_mpc = 0;
  double max_prone_speed = 0.0;
  double max_prone_joint_speed = 0.0;
  double max_prone_contact_force = 0.0;
  double max_prone_height_after_lowering = 0.0;
  double max_prone_upward_speed_after_lowering = 0.0;
  double max_prone_abs_pitch_after_lowering = 0.0;
  double completion_x = 0.0;
  double completion_y = 0.0;
  double maximum_hold_xy_displacement = 0.0;
  double maximum_hold_xy_speed = 0.0;
  std::array<int, kNumLegs> completion_foot_contact{};
  int completion_trunk_contact = 0;
  double completion_vx = 0.0;
  double completion_vy = 0.0;
  double completion_joint_error = 0.0;
  bool completion_sampled = false;
  bool reached_lowering_height = false;
  const int trunk = mj_name2id(model.get(), mjOBJ_BODY, "trunk");
  const auto observe_prone_dynamics = [&]() {
    const double body_height = data->xpos[3 * trunk + 2];
    if (!reached_lowering_height && body_height < 0.15) {
      reached_lowering_height = true;
    }
    if (reached_lowering_height) {
      max_prone_height_after_lowering = std::max(
        max_prone_height_after_lowering, body_height);
      max_prone_upward_speed_after_lowering = std::max(
        max_prone_upward_speed_after_lowering, std::max(0.0, data->qvel[2]));
      const mjtNum * rotation = data->xmat + 9 * trunk;
      max_prone_abs_pitch_after_lowering = std::max(
        max_prone_abs_pitch_after_lowering, std::abs(std::atan2(
          -rotation[6], std::sqrt(rotation[0] * rotation[0] + rotation[3] * rotation[3]))));
    }
    const auto foot_trace = readFootTrace(model.get(), data.get());
    for (float force : foot_trace.force) {
      max_prone_contact_force = std::max(max_prone_contact_force, static_cast<double>(force));
    }
    const double base_speed_squared = data->qvel[0] * data->qvel[0] +
      data->qvel[1] * data->qvel[1] + data->qvel[2] * data->qvel[2];
    max_prone_speed = std::max(max_prone_speed, base_speed_squared);
    for (int dof = 6; dof < model->nv; ++dof) {
      max_prone_joint_speed = std::max(max_prone_joint_speed, std::abs(data->qvel[dof]));
    }
  };
  for (std::size_t step_index = 0; step_index < 6000; ++step_index) {
    ASSERT_TRUE(step(
      model.get(), data.get(), runner, actuator_writer, diagnostics,
      kRlHeight, Vec3<float>::Zero(), false));
    observe_prone_dynamics();
    if (!entered_prone_mpc && runner.currentStateName() == FSM_StateName::LIE_DOWN) {
      entered_prone_mpc = true;
      rl_calls_at_prone_mpc = rl_policy_calls;
    }
    if (runner.proneDownComplete() &&
      runner.currentStateName() == FSM_StateName::LIE_DOWN)
    {
      completion_x = data->qpos[0];
      completion_y = data->qpos[1];
      const auto completion_trace = readFootTrace(model.get(), data.get());
      completion_foot_contact = completion_trace.contact;
      completion_vx = data->qvel[0];
      completion_vy = data->qvel[1];
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const auto & joint_model = runner.quadruped().leg(static_cast<LegId>(leg)).joints;
        for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
          const char * joint_name = nullptr;
          constexpr std::array<const char *, kNumJoints> joint_names{
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
          joint_name = joint_names[leg * kJointsPerLeg + joint];
          const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, joint_name);
          completion_joint_error = std::max(completion_joint_error, std::abs(
            data->qpos[model->jnt_qposadr[joint_id]]));
        }
        (void)joint_model;
      }
      const int trunk_geom = mj_name2id(model.get(), mjOBJ_GEOM, "trunk_collision");
      const int floor_geom = mj_name2id(model.get(), mjOBJ_GEOM, "floor");
      for (int contact_index = 0; contact_index < data->ncon; ++contact_index) {
        const auto & contact = data->contact[contact_index];
        if ((contact.geom1 == trunk_geom && contact.geom2 == floor_geom) ||
          (contact.geom1 == floor_geom && contact.geom2 == trunk_geom))
        {
          ++completion_trunk_contact;
        }
      }
      completion_sampled = true;
      break;
    }
  }
  ASSERT_TRUE(entered_prone_mpc);
  EXPECT_TRUE(runner.proneDownComplete());
  EXPECT_EQ(runner.currentStateName(), FSM_StateName::LIE_DOWN);
  EXPECT_EQ(rl_policy_calls, rl_calls_at_prone_mpc);
  EXPECT_LT(data->qpos[2], 0.20);
  for (int settle_step = 0; settle_step < 1000; ++settle_step) {
    ASSERT_TRUE(step(
      model.get(), data.get(), runner, actuator_writer, diagnostics,
      kRlHeight, Vec3<float>::Zero(), false));
    observe_prone_dynamics();
    ASSERT_TRUE(completion_sampled);
    const double hold_dx = data->qpos[0] - completion_x;
    const double hold_dy = data->qpos[1] - completion_y;
    maximum_hold_xy_displacement = std::max(
      maximum_hold_xy_displacement, std::sqrt(hold_dx * hold_dx + hold_dy * hold_dy));
    maximum_hold_xy_speed = std::max(
      maximum_hold_xy_speed, std::sqrt(data->qvel[0] * data->qvel[0] +
      data->qvel[1] * data->qvel[1]));
  }
  std::cout << "prone hold baseline: completion_xy=(" << completion_x << "," <<
    completion_y << ") max_xy_displacement=" << maximum_hold_xy_displacement <<
    " max_xy_speed=" << maximum_hold_xy_speed << " completion_xy_speed=(" <<
    completion_vx << "," << completion_vy << ") foot_contact=(" <<
    completion_foot_contact[0] << "," << completion_foot_contact[1] << "," <<
    completion_foot_contact[2] << "," << completion_foot_contact[3] << ") trunk_contact=" <<
    completion_trunk_contact << " max_joint_error=" << completion_joint_error << '\n';
  EXPECT_LT(max_prone_height_after_lowering, 0.17);
  EXPECT_LT(max_prone_upward_speed_after_lowering, 0.75);
  EXPECT_LT(max_prone_abs_pitch_after_lowering, 0.20);
  EXPECT_LT(max_prone_contact_force, 700.0);
  EXPECT_LT(std::sqrt(max_prone_speed), 0.75);
  EXPECT_LT(max_prone_joint_speed, 12.0);
  EXPECT_LT(maximum_hold_xy_displacement, 0.05);
  EXPECT_LT(maximum_hold_xy_speed, 0.15);
  // LieDown must end at the same motor-zero pose as the MuJoCo initialization
  // keyframe, while the staged fold prevents that pose from being commanded
  // as a sudden all-leg jump.
  EXPECT_NEAR(data->xpos[3 * trunk + 2], 0.12, 0.02);
  for (int leg = 0; leg < static_cast<int>(kNumLegs); ++leg) {
    const int qpos = 7 + leg * static_cast<int>(kJointsPerLeg);
    EXPECT_NEAR(data->qpos[qpos], 0.0, 0.08);
    EXPECT_NEAR(data->qpos[qpos + 1], 0.0, 0.08);
    EXPECT_NEAR(data->qpos[qpos + 2], 0.0, 0.08);
  }
}

TEST(Dm1Sim2Sim, OperatorArbiterEndToEndKeepsMotionAndFourFootSupport)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter actuator_writer(model.get());
  teleop::OperatorCommandArbiter arbiter;

  // Exercise the same command boundary used by SimulationBridge rather than
  // calling RobotRunner::requestStandUp() directly.
  arbiter.apply(*teleop::decodeKey('U'));
  ASSERT_EQ(arbiter.state(), teleop::MotorOutputState::Enabling);
  ASSERT_TRUE(arbiter.takeEnableRequest());
  arbiter.markEnabled();
  ASSERT_EQ(arbiter.state(), teleop::MotorOutputState::Enabled);
  arbiter.apply(*teleop::decodeKey('1'));

  bool control_active = false;
  bool stand_up_pending = false;
  bool reached_balance_stand = false;
  for (std::size_t step_index = 0; step_index < kStartupStepLimit; ++step_index) {
    if (arbiter.takeStandUpRequest()) {
      runner.prepareForMotionControl();
      control_active = true;
      stand_up_pending = true;
    }
    runner.setStandingHeight(kGenericStandHeight);
    const bool control_valid = !control_active || runner.run();
    ASSERT_TRUE(control_valid);
    if (stand_up_pending && runner.requestStandUp()) {stand_up_pending = false;}
    actuator_writer.write(runner, data.get(), control_active);
    mj_step(model.get(), data.get());
    if (control_active && !stand_up_pending &&
      runner.currentStateName() == FSM_StateName::BALANCE_STAND &&
      data->time >= 2.5 && genericStandStable(model.get(), data.get()))
    {
      reached_balance_stand = true;
      break;
    }
  }
  ASSERT_TRUE(reached_balance_stand);

  std::array<int, kNumLegs> stand_contact_samples{};
  std::array<float, kNumLegs> stand_normal_force{};
  for (int index = 0; index < 100; ++index) {
    ASSERT_TRUE(runner.run());
    actuator_writer.write(runner, data.get(), true);
    const auto trace = readFootTrace(model.get(), data.get());
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      stand_contact_samples[leg] += trace.contact[leg];
      stand_normal_force[leg] += trace.normal_force[leg];
    }
    mj_step(model.get(), data.get());
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_EQ(stand_contact_samples[leg], 100) << "leg=" << leg;
    EXPECT_GT(stand_normal_force[leg] / 100.0F, 1.0F) << "leg=" << leg;
  }

  // One terminal-style W command is intentionally held for more than the old
  // 500 ms timeout. It must continue until Space/Stop, not depend on repeats.
  arbiter.apply(*teleop::decodeKey('W'));
  ASSERT_TRUE(arbiter.motionActive());
  std::array<int, kNumLegs> motion_contact_samples{};
  std::array<float, kNumLegs> motion_normal_force{};
  const double initial_x = data->qpos[0];
  for (int index = 0; index < 1000; ++index) {  // 2 seconds at 500 Hz
    ASSERT_TRUE(arbiter.motionActive());
    const auto velocity = teleop::velocityForMotion(arbiter.motion());
    runner.setLocomotionVelocityCommand(velocity.forward, velocity.lateral, velocity.yaw);
    runner.setControlMode(ControlMode::Locomotion);
    ASSERT_TRUE(runner.run());
    actuator_writer.write(runner, data.get(), true);
    const auto trace = readFootTrace(model.get(), data.get());
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      motion_contact_samples[leg] += trace.contact[leg];
      motion_normal_force[leg] += trace.normal_force[leg];
    }
    mj_step(model.get(), data.get());
  }
  EXPECT_GT(data->qpos[0] - initial_x, 0.01);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_GT(motion_contact_samples[leg], 0) << "leg=" << leg;
    EXPECT_GT(motion_normal_force[leg], 1.0F) << "leg=" << leg;
  }
  arbiter.apply(*teleop::decodeKey(' '));
  EXPECT_FALSE(arbiter.motionActive());
  EXPECT_TRUE(arbiter.takeStopRequest());
}

TEST(Dm1Sim2Sim, LockedActuatorWriterClearsAllMuJoCoOutputs)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter actuator_writer(model.get());
  std::fill_n(data->ctrl, model->nu, 1.0);
  std::fill_n(data->qfrc_applied, model->nv, 1.0);
  actuator_writer.write(runner, data.get(), false);

  for (int actuator = 0; actuator < model->nu; ++actuator) {
    EXPECT_EQ(data->ctrl[actuator], 0.0);
  }
  for (int dof = 0; dof < model->nv; ++dof) {
    EXPECT_EQ(data->qfrc_applied[dof], 0.0);
  }
}

TEST(Dm1Sim2Sim, MotionControlStartsFromMeasuredJointPositionWithoutTorqueStep)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter actuator_writer(model.get());
  // Establish an output-gated initialization reference at the original q.
  ASSERT_TRUE(runner.run());

  constexpr std::array<const char *, kNumJoints> joint_names{
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
  std::array<Vec3<float>, kNumLegs> measured_joint_positions{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const char * name = joint_names[leg * kJointsPerLeg + joint];
      const int joint_id = mj_name2id(model.get(), mjOBJ_JOINT, name);
      ASSERT_GE(joint_id, 0);
      const int qpos_address = model->jnt_qposadr[joint_id];
      measured_joint_positions[leg](static_cast<Eigen::Index>(joint)) =
        static_cast<float>(data->qpos[qpos_address]) + 0.01F;
      data->qpos[qpos_address] = measured_joint_positions[leg](
        static_cast<Eigen::Index>(joint));
      data->qvel[model->jnt_dofadr[joint_id]] = 0.0;
    }
  }
  for (const char * name : joint_names) {
    const int joint = mj_name2id(model.get(), mjOBJ_JOINT, name);
    ASSERT_GE(joint, 0);
  }
  mj_forward(model.get(), data.get());

  // The existing output-gated trajectory still refers to the old q and would
  // inject a position step if the motor gate opened now.
  ASSERT_TRUE(runner.run());
  actuator_writer.write(runner, data.get(), true);
  double stale_peak_torque = 0.0;
  for (const char * name : joint_names) {
    const int joint = mj_name2id(model.get(), mjOBJ_JOINT, name);
    stale_peak_torque = std::max(
      stale_peak_torque,
      std::abs(data->qfrc_applied[model->jnt_dofadr[joint]]));
  }
  EXPECT_GT(stale_peak_torque, 0.5);

  // This reset is performed by the explicit Stand command, never by motor
  // unlock. It makes the first motion-control frame position-continuous.
  runner.prepareForMotionControl();
  ASSERT_TRUE(runner.run());
  actuator_writer.write(runner, data.get(), true);
  for (const char * name : joint_names) {
    const int joint = mj_name2id(model.get(), mjOBJ_JOINT, name);
    EXPECT_NEAR(data->qfrc_applied[model->jnt_dofadr[joint]], 0.0, 1.0e-5);
  }

  for (int step_index = 0; step_index < 700; ++step_index) {
    actuator_writer.write(runner, data.get(), true);
    mj_step(model.get(), data.get());
    ASSERT_TRUE(runner.run());
  }
  // The keyframe is deliberately prone, so the body may settle/slide while
  // the captured joints are held. The invariant under test is the command
  // continuity, not xy station keeping before StandUp.
  EXPECT_TRUE(std::isfinite(data->qpos[0]));
  EXPECT_TRUE(std::isfinite(data->qpos[1]));
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & command = runner.jointCommands()[leg];
    EXPECT_TRUE(command.position_desired.isApprox(measured_joint_positions[leg], 1.0e-5F));
    EXPECT_TRUE(command.kp.isApprox(Vec3<float>(100.0F, 100.0F, 100.0F)));
    EXPECT_TRUE(command.kd.isApprox(Vec3<float>(2.0F, 2.0F, 2.0F)));
  }

  ASSERT_EQ(runner.currentStateName(), FSM_StateName::JOINT_PD);
  ASSERT_TRUE(runner.requestStandUp());
  ASSERT_TRUE(runner.run());
  EXPECT_EQ(runner.currentStateName(), FSM_StateName::STAND_UP);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_TRUE(runner.jointCommands()[leg].position_desired.isApprox(
      measured_joint_positions[leg], 1.0e-5F));
  }
}
