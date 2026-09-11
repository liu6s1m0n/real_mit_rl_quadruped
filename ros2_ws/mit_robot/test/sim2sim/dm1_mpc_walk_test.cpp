/**
 * @file dm1_mpc_walk_diagnostic_test.cpp
 * @brief 无界面回归：MPC/WBC 行走必须连续推进，不能一步一停或反复退回站立。
 *
 * 该测试只读取仿真状态，不修改控制器行为；用于守住
 * locomotion_joint_kp/kd（WBC 零空间关节 PD）这一组行走稳定性参数。
 */
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

namespace
{
using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;

struct WalkReport
{
  std::size_t locomotion_entries = 0;
  std::size_t fallback_count = 0;
  std::size_t frames_in_locomotion = 0;
  double net_x = 0.0;
  double net_y = 0.0;
  double maximum_height = 0.0;
  double minimum_height = 1.0e9;
  double maximum_abs_pitch = 0.0;
  double maximum_joint_torque = 0.0;
  double maximum_joint_speed = 0.0;
};

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
    if (motion_started) {runner.setStandingHeight(0.39F);}
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
    // 等待机身真正抬到名义站高并静止，再发送方向命令；否则会把站起来
    // 过程本身的瞬态误判成行走不稳定。
    const double height = data->xpos[3 * trunk + 2];
    const double speed = std::sqrt(
      data->qvel[0] * data->qvel[0] + data->qvel[1] * data->qvel[1] +
      data->qvel[2] * data->qvel[2]);
    if (std::abs(height - 0.39) < 0.02 && speed < 0.05) {
      ++settled_steps;
      if (settled_steps >= 500) {return true;}
    } else {
      settled_steps = 0;
    }
  }
  return false;
}

WalkReport runDirection(
  const mjModel * model, mjData * data, RobotRunner & runner,
  SimulationActuatorWriter & writer, const Vec3<float> & command,
  std::size_t steps, double push_force_n = 0.0)
{
  const int trunk = mj_name2id(model, mjOBJ_BODY, "trunk");
  const double start_x = data->qpos[0];
  const double start_y = data->qpos[1];
  WalkReport report;
  FSM_StateName previous = runner.currentStateName();
  for (std::size_t index = 0; index < steps; ++index) {
    // 2 s 后施加 0.10 s 的横向外力，检查接触切换之外的扰动恢复能力。
    if (push_force_n != 0.0 && index >= 1000 && index < 1050) {
      data->xfrc_applied[6 * trunk + 1] = push_force_n;
    } else {
      data->xfrc_applied[6 * trunk + 1] = 0.0;
    }
    runner.setLocomotionVelocityCommand(command.x(), command.y(), command.z());
    runner.setControlMode(ControlMode::Locomotion);
    runner.run();
    writer.write(runner, data, true);
    const FSM_StateName state = runner.currentStateName();
    if (state != previous) {
      if (state == FSM_StateName::LOCOMOTION) {++report.locomotion_entries;}
      if (state == FSM_StateName::BALANCE_STAND &&
        previous == FSM_StateName::LOCOMOTION)
      {
        ++report.fallback_count;
      }
      previous = state;
    }
    if (state == FSM_StateName::LOCOMOTION) {++report.frames_in_locomotion;}
    report.maximum_height = std::max(report.maximum_height, data->xpos[3 * trunk + 2]);
    report.minimum_height = std::min(report.minimum_height, data->xpos[3 * trunk + 2]);
    const mjtNum * rotation = data->xmat + 9 * trunk;
    report.maximum_abs_pitch = std::max(report.maximum_abs_pitch, std::abs(
      std::atan2(-rotation[6],
      std::sqrt(rotation[0] * rotation[0] + rotation[3] * rotation[3]))));
    for (int dof = 6; dof < model->nv; ++dof) {
      report.maximum_joint_torque = std::max(
        report.maximum_joint_torque, std::abs(data->qfrc_applied[dof]));
      report.maximum_joint_speed = std::max(
        report.maximum_joint_speed, std::abs(data->qvel[dof]));
    }
    mj_step(model, data);
  }
  report.net_x = data->qpos[0] - start_x;
  report.net_y = data->qpos[1] - start_y;
  return report;
}

TEST(Dm1MpcWalkDiagnostic, AllDirectionsStayInLocomotion)
{
  struct Case
  {
    const char * name;
    Vec3<float> command;
  };
  const std::array<Case, 7> cases{{
      {"forward", Vec3<float>(0.18F, 0.0F, 0.0F)},
      {"forward_fast", Vec3<float>(0.36F, 0.0F, 0.0F)},
      {"backward", Vec3<float>(-0.30F, 0.0F, 0.0F)},
      {"left", Vec3<float>(0.0F, 0.20F, 0.0F)},
      {"right", Vec3<float>(0.0F, -0.20F, 0.0F)},
      {"rotate_ccw", Vec3<float>(0.0F, 0.0F, 0.30F)},
      {"rotate_cw", Vec3<float>(0.0F, 0.0F, -0.30F)}}};

  for (const auto & item : cases) {
    auto model = loadModel();
    auto data = makeData(model.get());
    RobotRunner runner(model.get(), data.get(), RobotType::DM1);
    SimulationActuatorWriter writer(model.get());
    ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer))
      << item.name << ": never reached BalanceStand";

    const auto report = runDirection(
      model.get(), data.get(), runner, writer, item.command, 6000);
    std::printf(
      "mpc_walk %-12s entries=%zu fallbacks=%zu frames=%zu net=(%.3f,%.3f) "
      "z=[%.3f,%.3f] max_pitch=%.3f max_tau=%.1f max_joint_speed=%.2f\n",
      item.name, report.locomotion_entries, report.fallback_count,
      report.frames_in_locomotion, report.net_x, report.net_y,
      report.minimum_height, report.maximum_height, report.maximum_abs_pitch,
      report.maximum_joint_torque, report.maximum_joint_speed);

    // 行走一旦进入 Locomotion 就必须连续保持；反复退回 BalanceStand 正是
    // 现场观察到的“一步一停”。
    EXPECT_EQ(report.fallback_count, 0U) << item.name;
    EXPECT_EQ(report.locomotion_entries, 1U) << item.name;
    EXPECT_GT(report.minimum_height, 0.30) << item.name;
    EXPECT_LT(report.maximum_height, 0.45) << item.name;
    EXPECT_LT(report.maximum_abs_pitch, 0.35) << item.name;
    // 仿真执行器峰值力矩与 DM1 的 ±97 Nm 契约一致，留出饱和裕度。
    EXPECT_LT(report.maximum_joint_torque, 97.0) << item.name;
    if (item.command.x() > 0.0F) {
      EXPECT_GT(report.net_x, 0.15) << item.name;
    } else if (item.command.x() < 0.0F) {
      EXPECT_LT(report.net_x, -0.15) << item.name;
    } else if (item.command.y() > 0.0F) {
      EXPECT_GT(report.net_y, 0.10) << item.name;
    } else if (item.command.y() < 0.0F) {
      EXPECT_LT(report.net_y, -0.10) << item.name;
    }
  }
}

// 横向扰动恢复：接触切换之外，机身姿态和关节阻抗也要能吸收外力，
// 否则提高 WBC 关节 PD 刚度就会把一次扰动变成持续振荡。
TEST(Dm1MpcWalkStress, ForwardWithLateralPush)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));

  const auto report = runDirection(
    model.get(), data.get(), runner, writer,
    Vec3<float>(0.18F, 0.0F, 0.0F), 4000, 120.0);
  std::printf(
    "mpc_walk push120N      entries=%zu fallbacks=%zu frames=%zu net=(%.3f,%.3f) "
    "z=[%.3f,%.3f] max_pitch=%.3f max_tau=%.1f max_joint_speed=%.2f\n",
    report.locomotion_entries, report.fallback_count,
    report.frames_in_locomotion, report.net_x, report.net_y,
    report.minimum_height, report.maximum_height, report.maximum_abs_pitch,
    report.maximum_joint_torque, report.maximum_joint_speed);
  EXPECT_EQ(report.fallback_count, 0U);
  EXPECT_GT(report.minimum_height, 0.30);
  EXPECT_LT(report.maximum_abs_pitch, 0.35);
  EXPECT_LT(report.maximum_joint_torque, 97.0);
}

// 入场门槛：只要 BalanceStand 一建立就立刻下发方向命令（真机上“按 1 后马上
// 按 W”的时序），控制器必须等机身抬到站立高度才切入 Locomotion，不能像
// 旧版本那样在 0.13 m 就开始迈步。
TEST(Dm1MpcWalkGate, LocomotionWaitsUntilTheBodyHasStoodUp)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  const int trunk = mj_name2id(model.get(), mjOBJ_BODY, "trunk");

  bool motion_started = false;
  bool stand_up_pending = false;
  bool entered_locomotion = false;
  double height_at_entry = 0.0;
  double entry_time = 0.0;
  double minimum_locomotion_height = 1.0e9;
  double start_x = 0.0;
  for (std::size_t index = 0; index < 20000; ++index) {
    if (!motion_started && data->time >= 1.3) {
      runner.prepareForMotionControl();
      motion_started = true;
      stand_up_pending = true;
    }
    if (motion_started) {runner.setStandingHeight(0.39F);}
    if (motion_started) {runner.run();}
    if (stand_up_pending && runner.requestStandUp()) {stand_up_pending = false;}
    // 只要进入 BalanceStand 就立刻请求行走，不等机身抬升完成。
    if (runner.currentStateName() == FSM_StateName::BALANCE_STAND) {
      runner.setLocomotionVelocityCommand(0.18F, 0.0F, 0.0F);
      runner.setControlMode(ControlMode::Locomotion);
    }
    writer.write(runner, data.get(), motion_started);
    const bool in_locomotion =
      runner.currentStateName() == FSM_StateName::LOCOMOTION;
    if (in_locomotion && !entered_locomotion) {
      entered_locomotion = true;
      height_at_entry = data->xpos[3 * trunk + 2];
      entry_time = data->time;
      start_x = data->qpos[0];
    }
    if (in_locomotion) {
      minimum_locomotion_height = std::min(
        minimum_locomotion_height, data->xpos[3 * trunk + 2]);
    }
    mj_step(model.get(), data.get());
    if (entered_locomotion && data->time - entry_time > 2.0) {break;}
  }
  std::printf(
    "mpc_walk gate         entered=%d height_at_entry=%.3f "
    "minimum_locomotion_height=%.3f net_x=%.3f\n",
    static_cast<int>(entered_locomotion), height_at_entry,
    minimum_locomotion_height, data->qpos[0] - start_x);
  // 门槛必须挡住趴卧高度（约 0.12 m）的起步请求。
  ASSERT_TRUE(entered_locomotion);
  EXPECT_GT(height_at_entry, 0.34);
  EXPECT_GT(minimum_locomotion_height, 0.34);
  EXPECT_GT(data->qpos[0] - start_x, 0.05);
}
}  // namespace
