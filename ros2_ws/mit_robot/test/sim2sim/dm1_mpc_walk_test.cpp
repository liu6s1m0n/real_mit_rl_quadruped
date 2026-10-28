/**
 * @file dm1_mpc_walk_diagnostic_test.cpp
 * @brief 无界面回归：MPC/WBC 行走必须连续推进，不能一步一停或反复退回站立。
 *
 * 该测试只读取仿真状态，不修改控制器行为；用于守住
 * joint_kp/kd（WBC 零空间关节 PD）这一组行走稳定性参数。
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
#include "teleop/operator_command.hpp"
#include "MPC/ConvexMPCLocomotion.h"
#include "WBC/WBC_Ctrl/BodyOriTask.hpp"
#include "model/floating_base_model_factory.hpp"
#include "model/robots/dm1.hpp"
#include "../fsm/fsm_test_support.hpp"
#include "FSM/FSM_State_Locomotion.h"

namespace
{
using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using DataPointer = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
constexpr std::array<std::array<const char *, kJointsPerLeg>, kNumLegs>
kJointNames{{
  {{"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint"}},
  {{"FL_hip_joint", "FL_thigh_joint", "FL_calf_joint"}},
  {{"RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"}},
  {{"RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"}}
}};

struct WalkReport
{
  std::size_t locomotion_entries = 0;
  std::size_t fallback_count = 0;
  std::size_t frames_in_locomotion = 0;
  double net_x = 0.0;
  double net_y = 0.0;
  double initial_reposition_x = 0.0;
  double initial_reposition_y = 0.0;
  double settled_delta_x = 0.0;
  double settled_delta_y = 0.0;
  double maximum_height = 0.0;
  double minimum_height = 1.0e9;
  double maximum_abs_pitch = 0.0;
  double accumulated_yaw = 0.0;
  double startup_maximum_planar_speed = 0.0;
  std::size_t startup_maximum_planar_speed_step = 0;
  double startup_maximum_abs_pitch = 0.0;
  double startup_maximum_joint_speed = 0.0;
  double startup_displacement_x = 0.0;
  double startup_displacement_y = 0.0;
  double maximum_joint_torque = 0.0;
  double maximum_raw_joint_torque = 0.0;
  double maximum_joint_speed = 0.0;
  double steady_maximum_joint_speed = 0.0;
  double absolute_torque_sum = 0.0;
  double squared_torque_sum = 0.0;
  std::size_t torque_samples = 0;
  std::size_t continuous_overload_samples = 0;
  std::size_t raw_overload_samples = 0;
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
    if (motion_started) {runner.setStandingHeight(0.28F);}
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
    if (std::abs(height - 0.28) < 0.02 && speed < 0.05) {
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
  std::array<std::array<int, kJointsPerLeg>, kNumLegs> joint_ids{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      joint_ids[leg][joint] =
        mj_name2id(model, mjOBJ_JOINT, kJointNames[leg][joint]);
    }
  }
  WalkReport report;
  double previous_yaw = std::atan2(data->xmat[9 * trunk + 3], data->xmat[9 * trunk]);
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
    const FSM_StateName state = runner.currentStateName();
    const auto & joint_commands = runner.jointCommands();
    if (state == FSM_StateName::LOCOMOTION) {
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
          const int joint_id = joint_ids[leg][joint];
          const int qpos = model->jnt_qposadr[joint_id];
          const int dof = model->jnt_dofadr[joint_id];
          const auto & joint_command = joint_commands[leg];
          const Eigen::Index component = static_cast<Eigen::Index>(joint);
          const double raw_torque = joint_command.torque_feedforward[component] +
            joint_command.kp[component] *
            (joint_command.position_desired[component] - data->qpos[qpos]) +
            joint_command.kd[component] *
            (joint_command.velocity_desired[component] - data->qvel[dof]);
          report.maximum_raw_joint_torque = std::max(
            report.maximum_raw_joint_torque, std::abs(raw_torque));
          if (std::abs(raw_torque) > 30.0 + 1.0e-4) {
            ++report.raw_overload_samples;
          }
        }
      }
    }
    writer.write(runner, data, true);
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
    if (index == steps / 2) {
      report.initial_reposition_x = data->qpos[0] - start_x;
      report.initial_reposition_y = data->qpos[1] - start_y;
    }
    report.maximum_height = std::max(report.maximum_height, data->xpos[3 * trunk + 2]);
    report.minimum_height = std::min(report.minimum_height, data->xpos[3 * trunk + 2]);
    const mjtNum * rotation = data->xmat + 9 * trunk;
    const double yaw = std::atan2(rotation[3], rotation[0]);
    const double yaw_delta = yaw - previous_yaw;
    report.accumulated_yaw += std::atan2(std::sin(yaw_delta), std::cos(yaw_delta));
    previous_yaw = yaw;
    const double abs_pitch = std::abs(std::atan2(-rotation[6],
      std::sqrt(rotation[0] * rotation[0] + rotation[3] * rotation[3])));
    report.maximum_abs_pitch = std::max(report.maximum_abs_pitch, abs_pitch);
    if (index < 1000) {
      const double planar_speed = std::hypot(data->qvel[0], data->qvel[1]);
      if (planar_speed > report.startup_maximum_planar_speed) {
        report.startup_maximum_planar_speed = planar_speed;
        report.startup_maximum_planar_speed_step = index;
      }
      report.startup_maximum_abs_pitch = std::max(
        report.startup_maximum_abs_pitch, abs_pitch);
      report.startup_displacement_x = data->qpos[0] - start_x;
      report.startup_displacement_y = data->qpos[1] - start_y;
    }
    for (int dof = 6; dof < model->nv; ++dof) {
      const double torque = data->qfrc_applied[dof];
      report.maximum_joint_torque = std::max(
        report.maximum_joint_torque, std::abs(torque));
      report.maximum_joint_speed = std::max(
        report.maximum_joint_speed, std::abs(data->qvel[dof]));
      if (index < 1000) {
        report.startup_maximum_joint_speed = std::max(
          report.startup_maximum_joint_speed, std::abs(data->qvel[dof]));
      } else {
        report.steady_maximum_joint_speed = std::max(
          report.steady_maximum_joint_speed, std::abs(data->qvel[dof]));
      }
      if (state == FSM_StateName::LOCOMOTION) {
        report.absolute_torque_sum += std::abs(torque);
        report.squared_torque_sum += torque * torque;
        ++report.torque_samples;
        if (std::abs(torque) > 30.0) {++report.continuous_overload_samples;}
      }
    }
    mj_step(model, data);
  }
  report.net_x = data->qpos[0] - start_x;
  report.net_y = data->qpos[1] - start_y;
  report.settled_delta_x = report.net_x - report.initial_reposition_x;
  report.settled_delta_y = report.net_y - report.initial_reposition_y;
  return report;
}

void printTorqueStats(const char * label, const WalkReport & report)
{
  const double samples = static_cast<double>(report.torque_samples);
  std::printf(
    "torque_stats %-12s mean_abs=%.3f rms=%.3f applied_peak=%.3f "
    "raw_peak=%.3f applied_over_30=%.4f%% raw_over_30=%zu\n",
    label, report.absolute_torque_sum / samples,
    std::sqrt(report.squared_torque_sum / samples), report.maximum_joint_torque,
    report.maximum_raw_joint_torque,
    100.0 * static_cast<double>(report.continuous_overload_samples) / samples,
    report.raw_overload_samples);
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
    printTorqueStats(item.name, report);
    std::printf(
      "startup %-12s displacement=(%.3f,%.3f) max_speed=%.3f step=%zu "
      "max_pitch=%.3f max_joint_speed=%.3f\n",
      item.name, report.startup_displacement_x,
      report.startup_displacement_y, report.startup_maximum_planar_speed,
      report.startup_maximum_planar_speed_step,
      report.startup_maximum_abs_pitch, report.startup_maximum_joint_speed);

    // 行走一旦进入 Locomotion 就必须连续保持；反复退回 BalanceStand 正是
    // 现场观察到的“一步一停”。
    EXPECT_EQ(report.fallback_count, 0U) << item.name;
    EXPECT_EQ(report.locomotion_entries, 1U) << item.name;
    EXPECT_GT(report.minimum_height, 0.26) << item.name;
    EXPECT_LT(report.maximum_height, 0.45) << item.name;
    EXPECT_LT(report.maximum_abs_pitch, 0.35) << item.name;
    EXPECT_EQ(report.raw_overload_samples, 0U) << item.name;
    EXPECT_LE(report.maximum_raw_joint_torque, 30.0 + 1.0e-3) << item.name;
    EXPECT_LE(report.maximum_joint_torque, 30.0 + 1.0e-3) << item.name;
    if (item.command.x() > 0.0F) {
      EXPECT_GT(report.net_x, 0.15) << item.name;
    } else if (item.command.x() < 0.0F) {
      EXPECT_LT(report.net_x, -0.15) << item.name;
    } else if (item.command.y() > 0.0F) {
      EXPECT_GT(report.net_y, 0.10) << item.name;
    } else if (item.command.y() < 0.0F) {
      EXPECT_LT(report.net_y, -0.10) << item.name;
    } else if (item.command.z() != 0.0F) {
      EXPECT_GT(report.accumulated_yaw * std::copysign(1.0F, item.command.z()),
        0.1) << item.name << ": turning command must produce heading motion";
    }
  }
}

TEST(Dm1MpcWalkDiagnostic, TrotWalkInPlaceStaysBalanced)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));

  runner.setLocomotionGait(GaitType::TROT);
  const auto report = runDirection(
    model.get(), data.get(), runner, writer,
    Vec3<float>(0.0F, 0.0F, 0.0F), 6000);
  std::printf(
    "mpc_walk march       entries=%zu fallbacks=%zu frames=%zu net=(%.3f,%.3f) "
    "z=[%.3f,%.3f] max_pitch=%.3f max_tau=%.1f max_joint_speed=%.2f\n",
    report.locomotion_entries, report.fallback_count,
    report.frames_in_locomotion, report.net_x, report.net_y,
    report.minimum_height, report.maximum_height, report.maximum_abs_pitch,
    report.maximum_joint_torque, report.maximum_joint_speed);
  printTorqueStats("march", report);
  std::printf(
    "march_windows initial_reposition=(%.3f,%.3f) settled_delta=(%.3f,%.3f)\n",
    report.initial_reposition_x, report.initial_reposition_y,
    report.settled_delta_x, report.settled_delta_y);
  std::printf(
    "march_startup displacement=(%.3f,%.3f) max_planar_speed=%.3f step=%zu "
    "max_pitch=%.3f max_joint_speed=%.3f\n",
    report.startup_displacement_x, report.startup_displacement_y,
    report.startup_maximum_planar_speed,
    report.startup_maximum_planar_speed_step, report.startup_maximum_abs_pitch,
    report.startup_maximum_joint_speed);

  EXPECT_EQ(report.fallback_count, 0U);
  EXPECT_EQ(report.locomotion_entries, 1U);
  EXPECT_GT(report.frames_in_locomotion, 5500U);
  EXPECT_GT(report.minimum_height, 0.26);
  EXPECT_LT(report.maximum_height, 0.45);
  EXPECT_LT(report.maximum_abs_pitch, 0.35);
  // 初次接触切换时不能立即把全部实测速度瞬态写入落脚捕获项。
  EXPECT_LT(report.startup_maximum_planar_speed, 0.30);
  EXPECT_LT(report.startup_maximum_abs_pitch, 0.10);
  // 选定的 TROT 摆动期为 0.15 s，稳态摆腿关节速度约 5 rad/s；入场瞬态
  // 不得明显超过稳态峰值，全程峰值须给 DM1 电机 2π rad/s 上限留 10% 余量。
  const double joint_speed_limit = static_cast<double>(
    robots::dm1::makeJointParameters<float>(false).velocity_limit.minCoeff());
  std::printf(
    "march_joint_speed startup=%.3f steady=%.3f limit=%.3f\n",
    report.startup_maximum_joint_speed, report.steady_maximum_joint_speed,
    joint_speed_limit);
  EXPECT_LT(report.startup_maximum_joint_speed,
    1.10 * report.steady_maximum_joint_speed);
  EXPECT_LT(report.maximum_joint_speed, 0.90 * joint_speed_limit);
  // 长时间运行后仍须回到完整反馈并保持收敛，不能持续漂移。
  EXPECT_LT(std::abs(report.initial_reposition_x), 0.20);
  EXPECT_LT(std::abs(report.initial_reposition_y), 0.20);
  EXPECT_LT(std::abs(report.settled_delta_x), 0.08);
  EXPECT_LT(std::abs(report.settled_delta_y), 0.08);
  EXPECT_EQ(report.raw_overload_samples, 0U);
  EXPECT_LE(report.maximum_raw_joint_torque, 30.0 + 1.0e-3);
  EXPECT_LE(report.maximum_joint_torque, 30.0 + 1.0e-3);
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
  printTorqueStats("push120N", report);
  EXPECT_EQ(report.fallback_count, 0U);
  EXPECT_GT(report.minimum_height, 0.26);
  EXPECT_LT(report.maximum_abs_pitch, 0.35);
  EXPECT_EQ(report.raw_overload_samples, 0U);
  EXPECT_LE(report.maximum_raw_joint_torque, 30.0 + 1.0e-3);
  EXPECT_LE(report.maximum_joint_torque, 30.0 + 1.0e-3);
}

// 入场门槛：只要 BalanceStand 一建立就立刻下发方向命令（真机上“按 1 后马上
// 按 W”的时序），控制器必须等高度指令到位并连续稳定后才
// 切入 Locomotion，不能在站起过程中开始迈步。
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
    if (motion_started) {runner.setStandingHeight(0.28F);}
    if (motion_started) {runner.run();}
    if (stand_up_pending && runner.requestStandUp()) {stand_up_pending = false;}
    // 只要进入 BalanceStand 就立刻请求行走，不等机身抬升完成。
    if (runner.currentStateName() == FSM_StateName::BALANCE_STAND ||
      runner.currentStateName() == FSM_StateName::LOCOMOTION)
    {
      // 与桥接层一致，在行走期间持续续发命令；入场 STAND 保持期会清零速度。
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
  // 仿真没有真机柔性下沉；高度指令走完后再计时，入场时机身
  // 应已接近 0.28 m 名义站高，而不是刚越过 0.26 m 下限。
  ASSERT_TRUE(entered_locomotion);
  EXPECT_GT(height_at_entry, 0.26);
  EXPECT_GT(minimum_locomotion_height, 0.25);
  EXPECT_GT(data->qpos[0] - start_x, 0.05);
}

TEST(Dm1MpcWalkGate, FoldedCalfCannotEnterLocomotion)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));
  ASSERT_TRUE(runner.locomotionEntryReady());

  const int joint = mj_name2id(model.get(), mjOBJ_JOINT, "FR_calf_joint");
  ASSERT_GE(joint, 0);
  data->qpos[model->jnt_qposadr[joint]] = 0.075;
  data->qvel[model->jnt_dofadr[joint]] = 0.0;
  mj_forward(model.get(), data.get());
  ASSERT_TRUE(runner.run());
  EXPECT_FALSE(runner.locomotionEntryReady());

  runner.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
  runner.setControlMode(ControlMode::Locomotion);
  ASSERT_TRUE(runner.run());
  EXPECT_EQ(runner.currentStateName(), FSM_StateName::BALANCE_STAND);
}

TEST(Dm1YawControl, DisabledYawLeavesHeadingFreeWhileCorrectingTilt)
{
  const auto robot = robots::dm1::makeModel<double>();
  auto model = model::makeFloatingBaseModel(robot);
  FBModelState<double> state;
  state.bodyPosition.setZero();
  state.bodyVelocity.setZero();
  state.q = DVec<double>::Zero(kNumJoints);
  state.qd = DVec<double>::Zero(kNumJoints);
  const Eigen::Quaterniond current =
    Eigen::AngleAxisd(0.7, Vec3<double>::UnitZ()) *
    Eigen::AngleAxisd(-0.12, Vec3<double>::UnitY()) *
    Eigen::AngleAxisd(0.08, Vec3<double>::UnitX());
  state.bodyOrientation << current.w(), current.x(), current.y(), current.z();
  model.setState(state);
  BodyOriTask<double> task(model);
  task.setKinematicGain(Vec3<double>(1.0, 1.0, 0.0));
  task.setProportionalGain(Vec3<double>(200.0, 200.0, 0.0));
  task.setDerivativeGain(Vec3<double>(20.0, 20.0, 0.0));
  ASSERT_TRUE(task.update(Eigen::Quaterniond::Identity()));
  DVec<double> reference;
  task.getCommand(reference);
  const DVec<double> reference_error = task.getPosError();
  EXPECT_GT(reference.head<2>().norm(), 1.0);
  EXPECT_DOUBLE_EQ(reference[2], 0.0);

  // 目标 yaw 和绕世界竖直轴的实测角速度均不得改变纠偏指令。
  state.bodyVelocity.head<3>() = Vec3<double>(0.1, -0.2, 0.4);
  model.setState(state);
  for (const double yaw : {-3.13, -1.0, 1.0, 3.13}) {
    const Eigen::Quaterniond desired =
      Eigen::AngleAxisd(yaw, Vec3<double>::UnitZ()) *
      Eigen::AngleAxisd(-0.12, Vec3<double>::UnitY()) *
      Eigen::AngleAxisd(0.08, Vec3<double>::UnitX());
    ASSERT_TRUE(task.update(desired));
    DVec<double> command;
    task.getCommand(command);
    EXPECT_DOUBLE_EQ(command[2], 0.0) << yaw;
    EXPECT_DOUBLE_EQ(task.getPosError()[2], 0.0) << yaw;
  }
}

TEST(Dm1YawControl, MpcDoesNotAccumulateHeadingErrorWhileMovingOrTurning)
{
  const auto robot = robots::dm1::makeModel<double>();
  mpc::ConvexMPCLocomotion<double> controller(robot, 0.002);
  EXPECT_DOUBLE_EQ(controller.solverSettings().state_weights[0], 0.0);
  EXPECT_DOUBLE_EQ(controller.solverSettings().state_weights[8], 0.0);
  GaitScheduler<double> scheduler(0.002);
  scheduler.step();
  StateEstimate<double> estimate;
  estimate.valid = true;
  estimate.position_world.z() = 0.28;
  DesiredState<double> desired;
  desired.valid = true;
  desired.body_position_world = estimate.position_world;
  desired.body_rpy.z() = 1.5;
  std::array<Vec3<double>, kNumLegs> feet{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    computeLegJacobianAndPosition(robot,
      robot.leg(static_cast<LegId>(leg)).joints.home_position,
      static_cast<Mat3<double> *>(nullptr), &feet[leg], static_cast<LegId>(leg));
    feet[leg] += estimate.position_world + robot.hipLocation(static_cast<LegId>(leg));
  }
  for (const double yaw_rate : {0.0, 0.3, -0.3}) {
    controller.setVelocityCommand(0.18, 0.0, yaw_rate);
    for (int step = 0; step < 60; ++step) {
      // 模拟航向漂移及 +/-pi 交界；不能追赶此前锁存或积分的航向。
      estimate.rpy.z() = step < 30 ? 3.13 : -3.13;
      estimate.orientation_world_from_body =
        Eigen::AngleAxisd(estimate.rpy.z(), Vec3<double>::UnitZ());
      estimate.rotation_world_from_body =
        estimate.orientation_world_from_body.toRotationMatrix();
      const auto result = controller.run(estimate, desired, feet,
        scheduler.gaitFrame(controller.solverSettings().horizon));
      ASSERT_TRUE(result.valid);
      EXPECT_NEAR(result.command.body_rpy.z(),
        estimate.rpy.z() + 0.002 * yaw_rate, 1e-12);
      EXPECT_DOUBLE_EQ(result.command.body_angular_velocity.z(), yaw_rate);
    }
  }
}

TEST(Dm1MpcWalkGate, TwoLegStandBypassesEntryGate)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));
  // 按键 3 的双腿实验入口可以直接接管当前站立控制。
  runner.setLocomotionGait(GaitType::TWO_LEG_STAND);
  runner.setControlMode(ControlMode::Locomotion);
  for (int step = 0; step < 3; ++step) {ASSERT_TRUE(runner.run());}
  EXPECT_EQ(runner.currentStateName(), FSM_StateName::LOCOMOTION);
}

void verifySingleDiagonalStep(int key, GaitType gait)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));
  teleop::OperatorCommandArbiter arbiter;
  arbiter.apply(teleop::decodeKey('U').value());
  arbiter.markEnabled();
  const std::array<const char *, kNumLegs> foot_names{
    "FR_foot", "FL_foot", "RR_foot", "RL_foot"};
  std::array<int, kNumLegs> foot_ids{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_ids[leg] = mj_name2id(model.get(), mjOBJ_BODY, foot_names[leg]);
    ASSERT_GE(foot_ids[leg], 0);
  }
  const int trunk = mj_name2id(model.get(), mjOBJ_BODY, "trunk");
  // 模拟正式桥接的锁存命令，落地回站后清除；同一个键再次按下可重做。
  for (int repeat = 0; repeat < 2; ++repeat) {
    SCOPED_TRACE(repeat);
    std::array<int, kNumLegs> liftoffs{}, touchdowns{};
    std::array<bool, kNumLegs> previous_contact{true, true, true, true};
    std::array<double, kNumLegs> initial_height{}, maximum_lift{};
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      initial_height[leg] = data->xpos[3 * foot_ids[leg] + 2];
    }
    int completion_step = -1;
    int first_swing_step = -1;
    int last_swing_step = -1;
    arbiter.apply(teleop::decodeKey(key).value());
    for (int step = 0; step < 800; ++step) {
      if (arbiter.motionActive()) {
        runner.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
        runner.setLocomotionGait(gait);
        runner.setControlMode(ControlMode::Locomotion);
      }
      const auto before = runner.currentStateName();
      // 测试执行器经 qfrc_applied 施加电机力矩；给 SimLeg 的反馈槽同步该值，
      // 避免把 ctrl=0 导致的 qfrc_actuator=0 误当成真实电机零力矩。
      mju_copy(data->qfrc_actuator, data->qfrc_applied, model->nv);
      ASSERT_TRUE(runner.run());
      const auto & scheduled = runner.gaitScheduler().gait_data;
      if (scheduled.current_gait == gait) {
        for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
          const bool contact = scheduled.contactScheduled(static_cast<LegId>(leg));
          liftoffs[leg] += previous_contact[leg] && !contact;
          touchdowns[leg] += !previous_contact[leg] && contact;
          previous_contact[leg] = contact;
          if (!contact) {
            if (first_swing_step < 0) {first_swing_step = step;}
            last_swing_step = step;
          }
        }
      }
      if (before == FSM_StateName::LOCOMOTION &&
        runner.currentStateName() == FSM_StateName::BALANCE_STAND && arbiter.motionActive())
      {
        arbiter.stopMotion();
        runner.setLocomotionVelocityCommand(0.0F, 0.0F, 0.0F);
        completion_step = step;
      }
      if (completion_step >= 0) {
        EXPECT_EQ(runner.currentStateName(), FSM_StateName::BALANCE_STAND);
      }
      writer.write(runner, data.get(), true);
      mj_step(model.get(), data.get());
      EXPECT_GT(data->xpos[3 * trunk + 2], 0.26);
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        maximum_lift[leg] = std::max(maximum_lift[leg],
          data->xpos[3 * foot_ids[leg] + 2] - initial_height[leg]);
      }
    }
    ASSERT_GE(first_swing_step, 0);
    EXPECT_NEAR((last_swing_step - first_swing_step + 1) * 0.002,
      control_config::gait::single_step_swing_time, 0.004);
    ASSERT_GE(completion_step, last_swing_step);
    EXPECT_LT(completion_step * 0.002,
      control_config::locomotion::entry_hold_duration +
      control_config::gait::single_step_swing_time +
      control_config::locomotion::touchdown_timeout_frames * 0.002 + 0.01);
    EXPECT_FALSE(arbiter.motionActive());
    EXPECT_EQ(runner.gaitScheduler().gait_data.current_gait, GaitType::STAND);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const bool swing = gait == GaitType::TWO_LEG_STAND ?
        (leg == 1 || leg == 2) : (leg == 0 || leg == 3);
      EXPECT_EQ(liftoffs[leg], swing ? 1 : 0);
      EXPECT_EQ(touchdowns[leg], swing ? 1 : 0);
      if (swing) {EXPECT_GT(maximum_lift[leg], 0.010);}
      EXPECT_LT(std::abs(data->xpos[3 * foot_ids[leg] + 2] - initial_height[leg]), 0.010);
    }
  }
}

TEST(Dm1MpcWalkDiagnostic, TwoLegStandStepsOnceLandsAndCanRepeat)
{
  verifySingleDiagonalStep('3', GaitType::TWO_LEG_STAND);
}

TEST(Dm1MpcWalkDiagnostic, TwoLegStandReverseStepsOnceLandsAndCanRepeat)
{
  verifySingleDiagonalStep('4', GaitType::TWO_LEG_STAND_REVERSE);
}

TEST(Dm1SingleStep, WaitsForMeasuredTouchdownAfterScheduledSwingEnds)
{
  for (const auto gait : {GaitType::TWO_LEG_STAND, GaitType::TWO_LEG_STAND_REVERSE}) {
    test_support::FsmContext context(0.002F);
    for (auto & joint : context.joint_states) {
      joint.valid = true;
      ASSERT_TRUE(context.leg_controller.updateData(joint));
    }
    context.estimate.position_world.z() = 0.28F;
    context.desired_state.body_position_world.z() = 0.28F;
    context.desired_state.mode = ControlMode::Locomotion;
    context.data().locomotion_gait = gait;
    context.data().use_wbc = false;
    context.data().measured_contact_valid.fill(true);
    context.data().measured_contact_probability.fill(0.0F);
    context.data().measured_normal_force.fill(0.0F);
    FSM_State_Locomotion<float> locomotion(&context.data());
    locomotion.onEnter();
    bool swing_finished = false;
    for (int frame = 0; frame < 400; ++frame) {
      context.gait_scheduler.step();
      context.data().gait_frame = context.gait_scheduler.gaitFrame(mpc::kDefaultMpcHorizon);
      ASSERT_EQ(locomotion.checkTransition(), FSM_StateName::LOCOMOTION);
      swing_finished = context.gait_scheduler.gait_data.current_gait == gait &&
        context.gait_scheduler.gait_data.initial_phase >= 1.0F;
      locomotion.run();
      context.estimate.timestamp += 0.002F;
      if (swing_finished) {break;}
    }
    ASSERT_TRUE(swing_finished);
    // 计划接触已恢复，实测接触尚未到来时仍不得自动回站。
    EXPECT_EQ(locomotion.checkTransition(), FSM_StateName::LOCOMOTION);
    context.data().measured_contact_probability.fill(1.0F);
    context.data().measured_normal_force.fill(context.quadruped.totalMass() * 9.81F / 4.0F);
    for (unsigned int frame = 0;
      frame < control_config::locomotion::touchdown_confirm_frames; ++frame)
    {
      ASSERT_EQ(locomotion.checkTransition(), FSM_StateName::LOCOMOTION);
      locomotion.run();
      context.estimate.timestamp += 0.002F;
    }
    EXPECT_EQ(locomotion.checkTransition(), FSM_StateName::BALANCE_STAND);
  }
}
}  // namespace
