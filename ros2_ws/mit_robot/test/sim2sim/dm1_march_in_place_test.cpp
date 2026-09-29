/**
 * @file dm1_march_in_place_test.cpp
 * @brief 原地踏步回归：2 号 TROT 与 3 号 STATIC_WALK 必须形成真实步态。
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
  double maximum_roll = 0.0;
  double maximum_pitch = 0.0;
  double net_x = 0.0;
  double net_y = 0.0;
};

MarchReport runMarch(
  const mjModel * model, mjData * data, RobotRunner & runner,
  SimulationActuatorWriter & writer,
  const std::function<GaitType(std::size_t)> & gait_at, std::size_t steps)
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
    runner.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
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
    const mjtNum * rotation = data->xmat + 9 * trunk;
    report.maximum_roll = std::max(
      report.maximum_roll, std::abs(std::atan2(rotation[7], rotation[8])));
    report.maximum_pitch = std::max(report.maximum_pitch, std::abs(std::atan2(
      -rotation[6], std::sqrt(rotation[0] * rotation[0] + rotation[3] * rotation[3]))));

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
    if (concurrent_swing >= 3) {++report.frames_with_three_or_more_swing;}
    ++report.frames;
    mj_step(model, data);
  }
  report.net_x = data->qpos[0] - start_x;
  report.net_y = data->qpos[1] - start_y;
  return report;
}

void printReport(const char * label, const MarchReport & report)
{
  std::printf(
    "%s lifts=[%zu %zu %zu %zu] foot_z=[%.3f %.3f %.3f %.3f] "
    "fallbacks=%zu max_concurrent_swing=%zu body_z=[%.3f,%.3f] "
    "roll=%.3f pitch=%.3f net=(%.3f,%.3f)\n",
    label,
    report.liftoffs[0], report.liftoffs[1], report.liftoffs[2], report.liftoffs[3],
    report.maximum_foot_height[0], report.maximum_foot_height[1],
    report.maximum_foot_height[2], report.maximum_foot_height[3],
    report.fallbacks, report.concurrent_swing_max,
    report.body_height_min, report.body_height_max,
    report.maximum_roll, report.maximum_pitch, report.net_x, report.net_y);
}

/** 所有原地踏步模式共用的健康检查：站稳、每条腿都按周期抬起、不触发回退。 */
void expectHealthyMarch(
  const MarchReport & report, std::size_t maximum_concurrent_swing,
  double maximum_net_x, double maximum_net_y)
{
  EXPECT_EQ(report.fallbacks, 0U);
  EXPECT_GT(report.body_height_min, 0.30);
  EXPECT_LT(report.body_height_max, 0.35);
  EXPECT_LT(report.maximum_roll, 0.15);
  EXPECT_LT(report.maximum_pitch, 0.15);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    // 0.5 s 周期内每腿抬起一次；允许 ±25% 的接触判定抖动。
    EXPECT_GT(report.liftoffs[leg], 30U) << kLegNames[leg];
    EXPECT_LT(report.liftoffs[leg], 50U) << kLegNames[leg];
    EXPECT_GT(report.maximum_foot_height[leg], 0.02) << kLegNames[leg];
  }
  EXPECT_LE(report.concurrent_swing_max, maximum_concurrent_swing);
  // 已知限制：纯速度环下原地踏步仍有低速漂移，这里只做宽松护栏。
  EXPECT_LT(std::abs(report.net_x), maximum_net_x);
  EXPECT_LT(std::abs(report.net_y), maximum_net_y);
}

constexpr std::size_t kMarchSteps = 10000;  // 20 s

/** 2 号原地对角小跑（TROT，50% 支撑）：对腿同步抬起，其余两腿支撑。 */
TEST(Dm1MarchInPlace, TrotStaysBalancedAndEveryLegSteps)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));

  const auto report = runMarch(
    model.get(), data.get(), runner, writer,
    [](std::size_t) {return GaitType::TROT;}, kMarchSteps);
  printReport("march_trot  ", report);

  expectHealthyMarch(report, 3U, 0.50, 0.30);
  EXPECT_LT(
    static_cast<double>(report.frames_with_three_or_more_swing) /
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

/** 3 号原地静态行走（STATIC_WALK，80% 支撑）：任意时刻最多一条腿摆动。 */
TEST(Dm1MarchInPlace, StaticWalkStaysBalancedAndEveryLegSteps)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));

  const auto report = runMarch(
    model.get(), data.get(), runner, writer,
    [](std::size_t) {return GaitType::STATIC_WALK;}, kMarchSteps);
  printReport("march_static", report);

  expectHealthyMarch(report, 1U, 0.60, 0.30);
}

/**
 * 步态分离：2 号与 3 号原地步态在行走中来回切换时，MPC 接触表与 GaitScheduler
 * 必须一起换相且不互相污染。这里 2s TROT -> 4s STATIC -> 2s TROT 连续切换。
 */
TEST(Dm1MarchInPlace, SwitchingBetweenInPlaceGaitsStaysBalanced)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));

  const auto report = runMarch(
    model.get(), data.get(), runner, writer,
    [](std::size_t index) {
      return index < 1000 || index >= 3000 ?
        GaitType::TROT : GaitType::STATIC_WALK;
    }, 4000);
  printReport("march_switch", report);

  EXPECT_EQ(report.fallbacks, 0U);
  EXPECT_GT(report.body_height_min, 0.30);
  EXPECT_LT(report.maximum_roll, 0.15);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_GT(report.liftoffs[leg], 10U) << kLegNames[leg];
  }
  EXPECT_LT(std::abs(report.net_x), 0.40);
  EXPECT_LT(std::abs(report.net_y), 0.30);
}
}  // namespace
