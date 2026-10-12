/**
 * @file dm1_march_in_place_test.cpp
 * @brief 原地踏步诊断：3 号入口四脚支撑；2 号与 TROT_WALK 保留摆腿回归。
 *
 * 现场问题：两个原地踏步模式都表现为腿部乱抖/机身前扑，看不出步态。该测试
 * 用无界面 MuJoCo 复现同一控制链，检查
 *   1) 四条腿都按步态周期反复抬起，而不是贴地不动或高频抖动；
 *   2) 机身高度、横滚保持在安全范围内，不触发安全回退；
 *   3) 原地命令下机身的水平漂移保持在已知限制以内。
 * 只读取仿真状态，不修改控制器行为。
 */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "RobotRunner.hpp"
#include "SimulationActuatorWriter.hpp"
#include "teleop/operator_command.hpp"
#include "FSM/FSM_State_Locomotion.h"
#include "model/floating_base_model_factory.hpp"
#include "../fsm/fsm_test_support.hpp"

namespace
{
using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

const std::array<const char *, kNumLegs> kLegNames{"FR", "FL", "RR", "RL"};

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

bool startStanding(
  const mjModel * model, mjData * data, RobotRunner & runner,
  SimulationActuatorWriter & writer)
{
  const int trunk = mj_name2id(model, mjOBJ_BODY, "trunk");
  bool motion_started = false;
  bool stand_up_pending = false;
  std::size_t settled_steps = 0;
  for (std::size_t index = 0; index < 20000; ++index) {
    if (!motion_started && data->time >= 1.3) {
      runner.prepareForMotionControl();
      motion_started = true;
      stand_up_pending = true;
    }
    if (motion_started) {runner.setStandingHeight(0.32F);}
    if (motion_started) {runner.run();}
    if (stand_up_pending && runner.requestStandUp()) {stand_up_pending = false;}
    writer.write(runner, data, motion_started);
    mj_step(model, data);
    if (stand_up_pending ||
      runner.currentStateName() != FSM_StateName::BALANCE_STAND)
    {
      settled_steps = 0;
      continue;
    }
    const double height = data->xpos[3 * trunk + 2];
    const double speed = std::sqrt(
      data->qvel[0] * data->qvel[0] + data->qvel[1] * data->qvel[1] +
      data->qvel[2] * data->qvel[2]);
    if (std::abs(height - 0.32) < 0.02 && speed < 0.05) {
      ++settled_steps;
      if (settled_steps >= 500) {return true;}
    } else {
      settled_steps = 0;
    }
  }
  return false;
}

struct MarchReport
{
  std::array<std::size_t, kNumLegs> liftoffs{};
  std::array<double, kNumLegs> maximum_foot_height{};
  std::size_t fallbacks = 0;
  std::size_t concurrent_swing_max = 0;
  std::size_t frames_with_three_or_more_swing = 0;
  std::size_t frames = 0;
  double body_height_min = 1.0e9;
  double body_height_max = -1.0e9;
  double startup_height_min = 1.0e9;
  double startup_height_max = -1.0e9;
  double startup_max_roll = 0.0;
  double startup_max_pitch = 0.0;
  double maximum_roll = 0.0;
  double maximum_pitch = 0.0;
  double net_x = 0.0;
  double net_y = 0.0;
  double initial_reposition_x = 0.0;
  double initial_reposition_y = 0.0;
  double settled_delta_x = 0.0;
  double settled_delta_y = 0.0;
};

MarchReport runMarch(
  const mjModel * model, mjData * data, RobotRunner & runner,
  SimulationActuatorWriter & writer,
  const std::function<GaitType(std::size_t)> & gait_at, std::size_t steps,
  teleop::VelocityCommand velocity = {})
{
  const int trunk = mj_name2id(model, mjOBJ_BODY, "trunk");
  const int floor_geom = mj_name2id(model, mjOBJ_GEOM, "floor");
  std::array<int, kNumLegs> foot_bodies{};
  std::array<int, kNumLegs> foot_geoms{};
  std::array<double, kNumLegs> base_z{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_bodies[leg] = mj_name2id(model, mjOBJ_BODY,
      (std::string(kLegNames[leg]) + "_foot").c_str());
    foot_geoms[leg] = mj_name2id(model, mjOBJ_GEOM, kLegNames[leg]);
    base_z[leg] = data->xpos[3 * foot_bodies[leg] + 2];
  }
  const double start_x = data->qpos[0];
  const double start_y = data->qpos[1];
  MarchReport report;
  std::array<bool, kNumLegs> contact{};
  FSM_StateName previous_state = runner.currentStateName();

  for (std::size_t index = 0; index < steps; ++index) {
    // 与 GUI 一致：每个控制周期下发一次步态选择，未变化时是空操作。
    runner.setLocomotionGait(gait_at(index));
    runner.setLocomotionVelocityCommand(velocity.forward, velocity.lateral, velocity.yaw);
    runner.setControlMode(ControlMode::Locomotion);
    runner.run();
    writer.write(runner, data, true);
    const FSM_StateName state = runner.currentStateName();
    if (state == FSM_StateName::BALANCE_STAND &&
      previous_state == FSM_StateName::LOCOMOTION)
    {
      ++report.fallbacks;
    }
    previous_state = state;

    const double body_z = data->xpos[3 * trunk + 2];
    report.body_height_min = std::min(report.body_height_min, body_z);
    report.body_height_max = std::max(report.body_height_max, body_z);
    if (index < 1000) {
      report.startup_height_min = std::min(report.startup_height_min, body_z);
      report.startup_height_max = std::max(report.startup_height_max, body_z);
    }
    const mjtNum * rotation = data->xmat + 9 * trunk;
    report.maximum_roll = std::max(
      report.maximum_roll, std::abs(std::atan2(rotation[7], rotation[8])));
    report.maximum_pitch = std::max(report.maximum_pitch, std::abs(std::atan2(
      -rotation[6], std::sqrt(rotation[0] * rotation[0] + rotation[3] * rotation[3]))));
    if (index < 1000) {
      report.startup_max_roll = std::max(
        report.startup_max_roll, std::abs(std::atan2(rotation[7], rotation[8])));
      report.startup_max_pitch = std::max(
        report.startup_max_pitch, std::abs(std::atan2(
          -rotation[6], std::sqrt(rotation[0] * rotation[0] + rotation[3] * rotation[3]))));
    }

    std::size_t concurrent_swing = 0;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const int foot = foot_bodies[leg];
      report.maximum_foot_height[leg] = std::max(
        report.maximum_foot_height[leg],
        data->xpos[3 * foot + 2] - base_z[leg]);
      bool now_contact = false;
      for (int c = 0; c < data->ncon; ++c) {
        const mjContact & ct = data->contact[c];
        if ((ct.geom1 == foot_geoms[leg] && ct.geom2 == floor_geom) ||
          (ct.geom2 == foot_geoms[leg] && ct.geom1 == floor_geom))
        {
          now_contact = true;
          break;
        }
      }
      if (contact[leg] && !now_contact) {++report.liftoffs[leg];}
      if (!now_contact) {++concurrent_swing;}
      contact[leg] = now_contact;
    }
    report.concurrent_swing_max = std::max(
      report.concurrent_swing_max, concurrent_swing);
    if (index == steps / 2) {
      report.initial_reposition_x = data->qpos[0] - start_x;
      report.initial_reposition_y = data->qpos[1] - start_y;
    }
    if (concurrent_swing >= 3) {++report.frames_with_three_or_more_swing;}
    ++report.frames;
    // 四脚诊断在首次回退时结束观察，避免把倒地后的无效命令计入阶段①。
    if (gait_at(index) == GaitType::STAND && report.fallbacks > 0) {break;}
    mj_step(model, data);
  }
  report.net_x = data->qpos[0] - start_x;
  report.net_y = data->qpos[1] - start_y;
  report.settled_delta_x = report.net_x - report.initial_reposition_x;
  report.settled_delta_y = report.net_y - report.initial_reposition_y;
  return report;
}

void printReport(const char * label, const MarchReport & report)
{
  std::printf(
    "%s lifts=[%zu %zu %zu %zu] foot_z=[%.3f %.3f %.3f %.3f] "
    "fallbacks=%zu max_concurrent_swing=%zu body_z=[%.3f,%.3f] "
    "roll=%.3f pitch=%.3f startup_z=[%.3f,%.3f] startup_roll=%.3f "
    "startup_pitch=%.3f net=(%.3f,%.3f) settled=(%.3f,%.3f)\n",
    label,
    report.liftoffs[0], report.liftoffs[1], report.liftoffs[2], report.liftoffs[3],
    report.maximum_foot_height[0], report.maximum_foot_height[1],
    report.maximum_foot_height[2], report.maximum_foot_height[3],
    report.fallbacks, report.concurrent_swing_max,
    report.body_height_min, report.body_height_max,
    report.maximum_roll, report.maximum_pitch, report.startup_height_min,
    report.startup_height_max, report.startup_max_roll, report.startup_max_pitch,
    report.net_x, report.net_y, report.settled_delta_x, report.settled_delta_y);
}

/** 所有原地踏步模式共用的健康检查：站稳、每条腿都按周期抬起、不触发回退。 */
void expectHealthyMarch(
  const MarchReport & report, std::size_t maximum_concurrent_swing,
  double maximum_net_x, double maximum_net_y,
  std::size_t minimum_liftoffs = 30, std::size_t maximum_liftoffs = 50)
{
  EXPECT_EQ(report.fallbacks, 0U);
  EXPECT_GT(report.body_height_min, 0.30);
  EXPECT_LT(report.body_height_max, 0.35);
  EXPECT_GT(report.startup_height_min, 0.30);
  EXPECT_LT(report.startup_height_max, 0.35);
  EXPECT_LT(report.maximum_roll, 0.15);
  EXPECT_LT(report.maximum_pitch, 0.15);
  EXPECT_LT(report.startup_max_roll, 0.15);
  EXPECT_LT(report.startup_max_pitch, 0.15);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    // 周期较慢的原地步态按调用方给出的窗口检查动作节奏。
    EXPECT_GT(report.liftoffs[leg], minimum_liftoffs) << kLegNames[leg];
    EXPECT_LT(report.liftoffs[leg], maximum_liftoffs) << kLegNames[leg];
    EXPECT_GT(report.maximum_foot_height[leg], 0.02) << kLegNames[leg];
  }
  EXPECT_LE(report.concurrent_swing_max, maximum_concurrent_swing);
  // 已知限制：纯速度环下原地踏步仍有低速漂移，这里只做宽松护栏。
  EXPECT_LT(std::abs(report.net_x), maximum_net_x);
  EXPECT_LT(std::abs(report.net_y), maximum_net_y);
}

constexpr std::size_t kMarchSteps = 10000;  // 20 s

TEST(Dm1MarchInPlace, Key3UsesUnmodifiedWalkingWbcWithFourContactsAndZeroVelocity)
{
  const auto command = teleop::decodeKey('3');
  ASSERT_TRUE(command.has_value());
  EXPECT_EQ(command->type, teleop::CommandType::Motion);
  EXPECT_EQ(command->motion, teleop::Motion::FourContactDiagnostic);
  const auto velocity = teleop::velocityForMotion(command->motion);
  EXPECT_FLOAT_EQ(velocity.forward, 0.0F);
  EXPECT_FLOAT_EQ(velocity.lateral, 0.0F);
  EXPECT_FLOAT_EQ(velocity.yaw, 0.0F);

  test_support::FsmContext context(0.002F);
  context.estimate.position_world.z() = 0.32F;
  context.estimate.velocity_world.x() = 0.08F;
  context.desired_state.mode = ControlMode::Locomotion;
  context.desired_state.body_position_world.z() = 0.32F;
  context.data().locomotion_gait = GaitType::STAND;
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }
  const auto & p = context.parameters;
  LocomotionCtrl<float> reference(model::makeFloatingBaseModel(context.quadruped));
  LegController<float> reference_commands(context.quadruped);
  reference.setBodyOrientationGains(p.locomotion_body_orientation_kp, p.locomotion_body_orientation_kd);
  reference.setJointGains(p.locomotion_joint_kp, p.locomotion_joint_kd);
  reference.setMaxNormalForce(p.maximum_normal_force);
  mpc::SolverSettings<float> settings;
  settings.time_step = 0.05F;
  mpc::ConvexMPCLocomotion<float> planner(context.quadruped, 0.002F, 20, settings, 25);
  planner.initialize();
  planner.setGait(GaitType::STAND);
  std::array<Vec3<float>, kNumLegs> feet{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    feet[leg] = context.estimate.position_world +
      context.quadruped.hipLocation(static_cast<LegId>(leg)) + context.leg_controller.datas[leg].p;
  }
  FSM_State_Locomotion<float> diagnostic(&context.data());
  diagnostic.onEnter();
  for (int frame = 0; frame < 1000; ++frame) {
    // 遗留非零速度不能让诊断模式运动；也不能让 MPC 恢复摆腿。
    diagnostic.setVelocityCommand(0.18F, 0.1F, 0.2F);
    context.gait_scheduler.step();
    const auto result = planner.run(context.estimate, context.desired_state, feet);
    ASSERT_TRUE(result.valid);
    LocomotionCtrlData<float> input;
    input.pBody_des = result.command.body_position_world;
    input.pBody_RPY_des = result.command.body_rpy;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      input.contact_state[leg] = 1.0F;
      input.Fr_des[leg] = result.reaction_forces_world[leg];
    }
    ASSERT_TRUE(reference.runAndApply(&input, context.estimate, context.joint_states, reference_commands));
    diagnostic.run();
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto id = static_cast<LegId>(leg);
      EXPECT_TRUE(context.gait_scheduler.gait_data.contactScheduled(id));
      EXPECT_FALSE(diagnostic.swingLegActive(id));
      const auto & actual = context.leg_controller.commands[leg];
      const auto & expected = reference_commands.commands[leg];
      EXPECT_TRUE(actual.torque_feedforward.isApprox(expected.torque_feedforward, 1.0e-5F));
      EXPECT_TRUE(actual.position_desired.isApprox(expected.position_desired, 1.0e-5F));
      EXPECT_TRUE(actual.velocity_desired.isApprox(expected.velocity_desired, 1.0e-5F));
      EXPECT_TRUE(actual.kp_joint.isApprox(p.locomotion_joint_kp));
      EXPECT_TRUE(actual.kd_joint.isApprox(p.locomotion_joint_kd));
    }
  }
}

/** 3 号入口阶段①的物理验收：沿用踏步控制，要求四脚贴地、机身稳定。 */
TEST(Dm1MarchInPlace, FourContactDiagnosticStaysGroundedWithoutAutomaticProgression)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));

  const auto report = runMarch(
    model.get(), data.get(), runner, writer,
    [](std::size_t) {return GaitType::STAND;}, kMarchSteps);
  printReport("four_contact", report);
  EXPECT_EQ(runner.currentStateName(), FSM_StateName::LOCOMOTION);
  EXPECT_EQ(report.fallbacks, 0U);
  EXPECT_EQ(report.concurrent_swing_max, 0U);
  EXPECT_GT(report.body_height_min, 0.30);
  EXPECT_LT(report.maximum_roll, 0.05);
  EXPECT_LT(report.maximum_pitch, 0.05);
  EXPECT_LT(std::abs(report.net_x), 0.02);
  EXPECT_LT(std::abs(report.net_y), 0.02);
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_EQ(report.liftoffs[leg], 0U);
    EXPECT_LT(report.maximum_foot_height[leg], 0.002);
    EXPECT_TRUE(runner.jointCommands()[leg].kp.isApprox(parameters.locomotion_joint_kp));
    EXPECT_TRUE(runner.jointCommands()[leg].kd.isApprox(parameters.locomotion_joint_kd));
  }
}

/** 2 号原地踏步保留原有 TROT_WALK（0.5 s/60% 支撑）执行行为。 */
TEST(Dm1MarchInPlace, TrotStaysBalancedAndEveryLegSteps)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));
  const auto report = runMarch(model.get(), data.get(), runner, writer,
    [](std::size_t) {return GaitType::TROT;}, kMarchSteps);
  printReport("march_trot", report);
  expectHealthyMarch(report, 3U, 0.50, 0.30, 20U, 45U);
  EXPECT_LT(static_cast<double>(report.frames_with_three_or_more_swing) /
    static_cast<double>(report.frames), 0.05);
}

/** 方向行走使用的 60% 对角小跑在零速度命令下也必须原地站稳。 */
TEST(Dm1MarchInPlace, TrotWalkStaysBalancedAndEveryLegSteps)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));

  const auto report = runMarch(
    model.get(), data.get(), runner, writer,
    [](std::size_t) {return GaitType::TROT_WALK;}, kMarchSteps);
  printReport("march_twalk ", report);

  expectHealthyMarch(report, 3U, 0.50, 0.30);
}

}  // namespace

TEST(Dm1MarchInPlace, InPlaceGaitSelectionDoesNotAffectDirectionKeys)
{
  constexpr char gait_key = '2';
  for (const char direction_key : {'w', 's', 'a', 'd', 'q', 'e'}) {
      SCOPED_TRACE(std::string("gait=2 direction=") + direction_key);
      teleop::OperatorCommandArbiter arbiter;
      arbiter.apply(*teleop::decodeKey('U'));
      arbiter.markEnabled();
      arbiter.applyBatch({*teleop::decodeKey(gait_key), *teleop::decodeKey(direction_key)});
      GaitType gait = GaitType::TROT_WALK;
      if (arbiter.motion() == teleop::Motion::MarchInPlace) {
        gait = GaitType::TROT;
      }
      EXPECT_EQ(gait, GaitType::TROT_WALK);
      auto model = loadModel();
      auto data = makeData(model.get());
      RobotRunner runner(model.get(), data.get(), RobotType::DM1);
      SimulationActuatorWriter writer(model.get());
      ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));
      const auto report = runMarch(model.get(), data.get(), runner, writer,
        [gait](std::size_t) {return gait;}, 1500, teleop::velocityForMotion(arbiter.motion()));
      printReport((std::string("direction_") + gait_key + direction_key).c_str(), report);
      EXPECT_EQ(report.fallbacks, 0U);
      EXPECT_GT(report.body_height_min, 0.25);
      EXPECT_LT(report.maximum_roll, 0.20);
      EXPECT_LT(report.maximum_pitch, 0.20);
  }
}
