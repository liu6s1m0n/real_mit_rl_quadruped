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
  std::array<double, kNumLegs> early_normal_force{};
  std::array<double, kNumLegs> late_normal_force{};
  std::array<double, kNumLegs> swing_normal_force{};
  std::array<bool, kNumLegs> final_contact{};
  std::size_t early_force_samples = 0;
  std::size_t late_force_samples = 0;
  std::size_t swing_force_samples = 0;
  std::size_t torque_limit_frames = 0;
  double rear_command_step_max = 0.0;
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
  teleop::VelocityCommand velocity = {},
  const std::function<void(std::size_t)> & before_control = {})
{
  const int trunk = mj_name2id(model, mjOBJ_BODY, "trunk");
  const int floor_geom = mj_name2id(model, mjOBJ_GEOM, "floor");
  std::array<int, kNumLegs> foot_bodies{};
  std::array<int, kNumLegs> foot_geoms{};
  std::array<int, 6> rear_joint_dofs{};
  std::array<double, kNumLegs> base_z{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    foot_bodies[leg] = mj_name2id(model, mjOBJ_BODY,
      (std::string(kLegNames[leg]) + "_foot").c_str());
    foot_geoms[leg] = mj_name2id(model, mjOBJ_GEOM, kLegNames[leg]);
    base_z[leg] = data->xpos[3 * foot_bodies[leg] + 2];
  }
  for (std::size_t rear = 0; rear < 2; ++rear) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto name = std::string(kLegNames[rear + 2]) +
        (joint == 0 ? "_hip_joint" : joint == 1 ? "_thigh_joint" : "_calf_joint");
      const int id = mj_name2id(model, mjOBJ_JOINT, name.c_str());
      rear_joint_dofs[rear * kJointsPerLeg + joint] = model->jnt_dofadr[id];
    }
  }
  const double start_x = data->qpos[0];
  const double start_y = data->qpos[1];
  MarchReport report;
  std::array<bool, kNumLegs> contact{};
  std::array<double, 6> previous_rear_torque{};
  FSM_StateName previous_state = runner.currentStateName();

  for (std::size_t index = 0; index < steps; ++index) {
    if (before_control) {before_control(index);}
    // 与 GUI 一致：每个控制周期下发一次步态选择，未变化时是空操作。
    runner.setLocomotionGait(gait_at(index));
    runner.setLocomotionVelocityCommand(velocity.forward, velocity.lateral, velocity.yaw);
    runner.setControlMode(ControlMode::Locomotion);
    runner.run();
    writer.write(runner, data, true);
    if (gait_at(index) == GaitType::STAND_CYCLE) {
      if (index < 250) {
        for (std::size_t rear = 0; rear < 2; ++rear) {
          for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
            const std::size_t slot = rear * kJointsPerLeg + joint;
            const double torque = data->qfrc_applied[rear_joint_dofs[slot]];
            if (index != 0) {
              report.rear_command_step_max = std::max(
                report.rear_command_step_max,
                std::abs(torque - previous_rear_torque[slot]));
            }
            previous_rear_torque[slot] = torque;
          }
        }
      }
      for (int dof = 6; dof < model->nv; ++dof) {
        if (std::abs(data->qfrc_applied[dof]) >= 59.0) {
          ++report.torque_limit_frames;
          break;
        }
      }
    }
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
    const bool early_force_window = index >= 100 && index < 200;
    const bool late_force_window = index + 700 >= steps && index + 300 < steps;
    const bool swing_force_window = index >= 455 && index < 485;
    report.early_force_samples += early_force_window ? 1U : 0U;
    report.late_force_samples += late_force_window ? 1U : 0U;
    report.swing_force_samples += swing_force_window ? 1U : 0U;
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
          if (early_force_window || late_force_window || swing_force_window) {
            mjtNum contact_force[6]{};
            mj_contactForce(model, data, c, contact_force);
            if (early_force_window) {
              report.early_normal_force[leg] += contact_force[0];
            } else if (late_force_window) {
              report.late_normal_force[leg] += contact_force[0];
            } else {
              report.swing_normal_force[leg] += contact_force[0];
            }
          }
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
  report.final_contact = contact;
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
  reference.setBodyPositionGains(p.balance_body_position_kp, p.balance_body_position_kd);
  reference.setBodyOrientationGains(p.locomotion_body_orientation_kp, p.locomotion_body_orientation_kd);
  reference.setFootPositionGains(
    Vec3<float>::Constant(500.0F), Vec3<float>::Constant(10.0F));
  reference.setJointGains(p.joint_kp, p.joint_kd);
  reference.setFloatingBaseWeight(3.0F);
  reference.setReactionForceWeight(1.0F);
  reference.setMaxNormalForce(p.maximum_normal_force);
  mpc::SolverSettings<float> settings;
  settings.time_step = 0.030F;
  mpc::ConvexMPCLocomotion<float> planner(context.quadruped, 0.002F, 15, settings, 15);
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
      EXPECT_TRUE(actual.torque_feedforward.allFinite());
      EXPECT_TRUE(actual.position_desired.allFinite());
      EXPECT_TRUE(actual.velocity_desired.allFinite());
      EXPECT_TRUE(actual.kp_joint.isApprox(p.joint_kp));
      EXPECT_TRUE(actual.kd_joint.isApprox(p.joint_kd));
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
  EXPECT_LT(std::abs(report.net_x), 0.03);
  EXPECT_LT(std::abs(report.net_y), 0.02);
  const auto parameters = makeRobotControlParameters<float>(RobotType::DM1);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_EQ(report.liftoffs[leg], 0U);
    EXPECT_LT(report.maximum_foot_height[leg], 0.002);
    EXPECT_TRUE(runner.jointCommands()[leg].kp.isApprox(parameters.joint_kp));
    EXPECT_TRUE(runner.jointCommands()[leg].kd.isApprox(parameters.joint_kd));
  }
}

TEST(Dm1MarchInPlace, Key4LiftsOnceThenRestoresFourLegSupport)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));
  const int trunk = mj_name2id(model.get(), mjOBJ_BODY, "trunk");
  const double start_z = data->xpos[3 * trunk + 2];
  std::array<int, kNumJoints> joint_qpos{};
  std::array<double, kNumJoints> start_q{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const auto name = std::string(kLegNames[leg]) +
        (joint == 0 ? "_hip_joint" : joint == 1 ? "_thigh_joint" : "_calf_joint");
      const std::size_t slot = leg * kJointsPerLeg + joint;
      joint_qpos[slot] = model->jnt_qposadr[mj_name2id(model.get(), mjOBJ_JOINT, name.c_str())];
      start_q[slot] = data->qpos[joint_qpos[slot]];
    }
  }

  const auto report = runMarch(model.get(), data.get(), runner, writer,
    [](std::size_t) {return GaitType::STAND_CYCLE;}, 5000);
  printReport("load_transfer", report);
  // 两脚支撑期间质心必须落在 FR-RL 支撑线上：不掉高、不绕对角线倾倒，
  // 抬落结束后姿态、关节角和水平位置都回到抬腿前。
  double joint_drift = 0.0;
  for (std::size_t slot = 0; slot < kNumJoints; ++slot) {
    joint_drift = std::max(joint_drift, std::abs(data->qpos[joint_qpos[slot]] - start_q[slot]));
  }
  std::printf("one_step height_drop=%.4f m joint_drift=%.4f rad end_z=%.4f\n",
    start_z - report.body_height_min, joint_drift, data->xpos[3 * trunk + 2]);
  EXPECT_LT(start_z - report.body_height_min, 0.01);
  EXPECT_LT(report.maximum_roll, 0.05);
  EXPECT_LT(report.maximum_pitch, 0.05);
  EXPECT_LT(joint_drift, 0.06);
  EXPECT_NEAR(data->xpos[3 * trunk + 2], start_z, 0.008);
  EXPECT_LT(std::abs(report.net_x), 0.01);
  EXPECT_LT(std::abs(report.net_y), 0.01);
  ASSERT_GT(report.early_force_samples, 0U);
  ASSERT_GT(report.late_force_samples, 0U);
  EXPECT_EQ(runner.currentStateName(), FSM_StateName::LOCOMOTION);
  EXPECT_EQ(report.fallbacks, 0U);
  EXPECT_GT(report.body_height_min, 0.30);
  EXPECT_LT(report.maximum_roll, 0.15);
  EXPECT_LT(report.maximum_pitch, 0.15);
  EXPECT_EQ(report.torque_limit_frames, 0U);
  EXPECT_LT(report.rear_command_step_max, 8.0);
  for (const std::size_t leg : {std::size_t(0), std::size_t(3)}) {
    EXPECT_EQ(report.liftoffs[leg], 0U) << kLegNames[leg];
    EXPECT_LT(report.maximum_foot_height[leg], 0.002) << kLegNames[leg];
  }
  for (const std::size_t leg : {std::size_t(1), std::size_t(2)}) {
    EXPECT_EQ(report.liftoffs[leg], 1U) << kLegNames[leg];
    EXPECT_GT(report.maximum_foot_height[leg], 0.01) << kLegNames[leg];
    EXPECT_LT(report.maximum_foot_height[leg], 0.03) << kLegNames[leg];
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_TRUE(report.final_contact[leg]) << kLegNames[leg];
  }
  const double early_unload =
    (report.early_normal_force[1] + report.early_normal_force[2]) /
    static_cast<double>(report.early_force_samples);
  const double late_unload =
    (report.late_normal_force[1] + report.late_normal_force[2]) /
    static_cast<double>(report.late_force_samples);
  const double early_support =
    (report.early_normal_force[0] + report.early_normal_force[3]) /
    static_cast<double>(report.early_force_samples);
  const double late_support =
    (report.late_normal_force[0] + report.late_normal_force[3]) /
    static_cast<double>(report.late_force_samples);
  const double swing_unload =
    (report.swing_normal_force[1] + report.swing_normal_force[2]) /
    static_cast<double>(report.swing_force_samples);
  const double swing_support =
    (report.swing_normal_force[0] + report.swing_normal_force[3]) /
    static_cast<double>(report.swing_force_samples);
  std::printf("one_step force unload %.1f -> %.1f -> %.1f N, support %.1f -> %.1f -> %.1f N\n",
    early_unload, swing_unload, late_unload, early_support, swing_support, late_support);
  std::printf("one_step torque_limit_frames=%zu rear_command_step_max=%.3f Nm\n",
    report.torque_limit_frames, report.rear_command_step_max);
  EXPECT_LT(swing_unload, 1.0);
  EXPECT_GT(swing_support, (early_unload + early_support) * 0.95);
  EXPECT_NEAR(late_unload, early_unload, 10.0);
  EXPECT_NEAR(late_support, early_support, 10.0);
}

TEST(Dm1MarchInPlace, Key4PlannerSchedulerAndEstimatorAgreeOnOneLift)
{
  test_support::FsmContext context(0.002F);
  context.estimate.position_world.z() = 0.32F;
  context.desired_state.body_position_world.z() = 0.32F;
  mpc::SolverSettings<float> settings;
  settings.time_step = 0.05F;
  mpc::ConvexMPCLocomotion<float> planner(context.quadruped, 0.002F, 20, settings, 25);
  planner.initialize();
  planner.setGait(GaitType::STAND_CYCLE);
  context.gait_scheduler.step();  // 已站立的入口接触状态。
  context.gait_scheduler.requestGait(GaitType::STAND_CYCLE);
  context.gait_scheduler.modifyGait();
  std::array<Vec3<float>, kNumLegs> feet{};
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    feet[leg] = context.estimate.position_world +
      context.quadruped.hipLocation(static_cast<LegId>(leg)) + context.leg_controller.datas[leg].p;
  }
  std::array<int, kNumLegs> swing_frames{};
  std::array<int, kNumLegs> liftoffs{};
  for (int frame = 0; frame < 2000; ++frame) {
    SCOPED_TRACE(frame);
    const auto result = planner.run(context.estimate, context.desired_state, feet);
    ASSERT_TRUE(result.valid);
    const auto probabilities = context.gait_scheduler.gait_data.estimatorContactProbabilities();
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto & gait = context.gait_scheduler.gait_data;
      const bool expected_contact = !(leg == 1 || leg == 2) || frame < 300 || frame >= 420;
      EXPECT_EQ(result.contact_state[leg], expected_contact);
      EXPECT_EQ(gait.contactScheduled(static_cast<LegId>(leg)), expected_contact);
      liftoffs[leg] += gait.liftoff_scheduled(leg);
      if (!expected_contact) {
        ++swing_frames[leg];
        EXPECT_FLOAT_EQ(probabilities[leg], 0.0F);
        EXPECT_NEAR(result.swing_phase[leg], gait.phase_swing(leg), 1.0e-4F);
        EXPECT_LT(result.reaction_forces_world[leg].norm(), 1.0e-3F);
      } else if (frame >= 520) {
        EXPECT_NEAR(probabilities[leg], 1.0F, 1.0e-4F);
      }
    }
    context.gait_scheduler.step();
  }
  EXPECT_EQ(swing_frames, (std::array<int, kNumLegs>{0, 120, 120, 0}));
  EXPECT_EQ(liftoffs, (std::array<int, kNumLegs>{0, 1, 1, 0}));
}

TEST(Dm1MarchInPlace, ZeroVelocityKeepsPositionReferenceAfterDisplacement)
{
  test_support::FsmContext context(0.002F);
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }
  context.estimate.position_world << 0.2F, -0.1F, 0.32F;
  context.desired_state.body_position_world.z() = 0.32F;
  mpc::SolverSettings<float> settings;
  settings.time_step = 0.05F;
  for (const auto gait : {GaitType::STAND, GaitType::STAND_CYCLE, GaitType::TROT_WALK}) {
    mpc::ConvexMPCLocomotion<float> planner(context.quadruped, 0.002F, 20, settings, 25);
    planner.initialize();
    planner.setGait(gait);
    std::array<Vec3<float>, kNumLegs> feet{};
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      feet[leg] = context.estimate.position_world +
        context.quadruped.hipLocation(static_cast<LegId>(leg)) + context.leg_controller.datas[leg].p;
    }
    const auto anchor = planner.run(context.estimate, context.desired_state, feet).command.body_position_world;
    context.estimate.position_world.x() += 0.04F;
    for (int frame = 0; frame < 80; ++frame) {
      const auto result = planner.run(context.estimate, context.desired_state, feet);
      ASSERT_TRUE(result.valid);
      EXPECT_TRUE(result.command.body_position_world.isApprox(anchor));
    }
    planner.setVelocityCommand(0.18F, 0.0F, 0.0F);
    const auto moving = planner.run(context.estimate, context.desired_state, feet);
    EXPECT_FLOAT_EQ(moving.command.body_position_world.x(), context.estimate.position_world.x());
    planner.setVelocityCommand(0.0F, 0.0F, 0.0F);
    for (int frame = 0; frame < 100; ++frame) {
      ASSERT_TRUE(planner.run(context.estimate, context.desired_state, feet).valid);
    }
    const float stop_x = context.estimate.position_world.x();
    context.estimate.position_world.x() += 0.03F;
    const auto stopped = planner.run(context.estimate, context.desired_state, feet);
    EXPECT_FLOAT_EQ(stopped.command.body_position_world.x(), stop_x);
  }
}

TEST(Dm1MarchInPlace, CalfGainAddsSupportTorqueWithTheSameWbcReference)
{
  test_support::FsmContext context(0.002F);
  for (auto & joint : context.joint_states) {
    joint.valid = true;
    ASSERT_TRUE(context.leg_controller.updateData(joint));
  }
  context.estimate.position_world.z() = 0.30F;
  const auto & p = context.parameters;
  LocomotionCtrlData<float> input;
  input.pBody_des.z() = 0.32F;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    input.contact_state[leg] = 1.0F;
    input.Fr_des[leg].z() = context.quadruped.totalMass() * 9.81F / kNumLegs;
  }
  std::array<float, 2> left_rear_calf_torque{};
  Vec3<float> old_reference = Vec3<float>::Zero();
  for (std::size_t mode = 0; mode < 2; ++mode) {
    LocomotionCtrl<float> wbc(model::makeFloatingBaseModel(context.quadruped));
    wbc.setBodyPositionGains(p.balance_body_position_kp, p.balance_body_position_kd);
    wbc.setBodyOrientationGains(p.locomotion_body_orientation_kp, p.locomotion_body_orientation_kd);
    Vec3<float> kp = p.joint_kp;
    if (mode == 0) {kp.z() = 65.0F;}
    wbc.setJointGains(kp, p.joint_kd);
    LegController<float> output(context.quadruped);
    ASSERT_TRUE(wbc.runAndApply(&input, context.estimate, context.joint_states, output));
    const auto & command = output.commands[3];
    const auto & state = context.joint_states[3];
    left_rear_calf_torque[mode] = command.torque_feedforward.z() +
      command.kp_joint.z() * (command.position_desired.z() - state.position.z()) +
      command.kd_joint.z() * (command.velocity_desired.z() - state.velocity.z());
    if (mode == 0) {old_reference = command.position_desired;}
    EXPECT_TRUE(command.position_desired.isApprox(old_reference));
  }
  std::printf("calf_support torque Kp65=%.3f Kp90=%.3f Nm\n",
    left_rear_calf_torque[0], left_rear_calf_torque[1]);
  EXPECT_GT(left_rear_calf_torque[0], left_rear_calf_torque[1] + 0.5F);
}

TEST(Dm1MarchInPlace, Key4RecoversFromAPhysicalPitchAndPositionDisturbance)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));
  const int trunk = mj_name2id(model.get(), mjOBJ_BODY, "trunk");
  std::size_t disturbed_frames = 0;
  const auto report = runMarch(model.get(), data.get(), runner, writer,
    [](std::size_t) {return GaitType::STAND_CYCLE;}, 5000, {},
    [&](std::size_t frame) {
      const bool disturb = frame >= 430 && frame < 530;
      data->xfrc_applied[6 * trunk] = disturb ? 4.0 : 0.0;
      data->xfrc_applied[6 * trunk + 3] = disturb ? -0.35 : 0.0;
      // 质心平移后原 2 Nm 只能压出约 0.013 rad，加大到 4 Nm 保证扰动可观测。
      data->xfrc_applied[6 * trunk + 4] = disturb ? -4.0 : 0.0;
      disturbed_frames += disturb ? 1U : 0U;
    });
  printReport("disturbed_one_step", report);
  EXPECT_EQ(disturbed_frames, 100U);
  EXPECT_EQ(report.fallbacks, 0U);
  EXPECT_EQ(report.torque_limit_frames, 0U);
  EXPECT_GT(report.body_height_min, 0.28);
  EXPECT_LT(report.maximum_roll, 0.2);
  EXPECT_LT(report.maximum_pitch, 0.2);
  EXPECT_GT(report.maximum_pitch, 0.02);
  const mjtNum * rotation = data->xmat + 9 * trunk;
  EXPECT_LT(std::abs(std::atan2(rotation[7], rotation[8])), 0.03);
  EXPECT_LT(std::abs(std::atan2(-rotation[6], std::hypot(rotation[0], rotation[3]))), 0.03);
  EXPECT_LT(std::abs(report.net_x), 0.03);
  EXPECT_LT(std::abs(report.net_y), 0.03);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_TRUE(report.final_contact[leg]);
    EXPECT_GT(report.late_normal_force[leg] / report.late_force_samples, 10.0);
    const auto & command = runner.jointCommands()[leg];
    const auto & home = runner.quadruped().leg(static_cast<LegId>(leg)).joints.home_position;
    EXPECT_LT(std::abs(command.position_desired.y() - home.y()), 0.25);
    EXPECT_LT(std::abs(command.position_desired.z() - home.z()), 0.25);
  }
}

TEST(Dm1MarchInPlace, Key4RightRearClearsGroundWithAnAdditionalDownwardLoad)
{
  auto model = loadModel();
  auto data = makeData(model.get());
  RobotRunner runner(model.get(), data.get(), RobotType::DM1);
  SimulationActuatorWriter writer(model.get());
  ASSERT_TRUE(startStanding(model.get(), data.get(), runner, writer));
  const int right_rear = mj_name2id(model.get(), mjOBJ_BODY, "RR_foot");
  const auto report = runMarch(model.get(), data.get(), runner, writer,
    [](std::size_t) {return GaitType::STAND_CYCLE;}, 5000, {},
    [&](std::size_t frame) {
      // 右后脚在换载、抬落阶段额外承受2 N向下外力，检查真实仿真接触。
      data->xfrc_applied[6 * right_rear + 2] = frame >= 250 && frame < 520 ? -2.0 : 0.0;
    });
  printReport("right_rear_loaded_one_step", report);
  EXPECT_EQ(report.fallbacks, 0U);
  EXPECT_EQ(report.torque_limit_frames, 0U);
  EXPECT_GT(report.body_height_min, 0.30);
  EXPECT_LT(report.maximum_roll, 0.15);
  EXPECT_LT(report.maximum_pitch, 0.15);
  for (const std::size_t leg : {std::size_t(1), std::size_t(2)}) {
    EXPECT_EQ(report.liftoffs[leg], 1U) << kLegNames[leg];
    EXPECT_GT(report.maximum_foot_height[leg], 0.01) << kLegNames[leg];
    EXPECT_LT(report.swing_normal_force[leg] / report.swing_force_samples, 1.0);
  }
  for (const std::size_t leg : {std::size_t(0), std::size_t(3)}) {
    EXPECT_EQ(report.liftoffs[leg], 0U) << kLegNames[leg];
  }
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_TRUE(report.final_contact[leg]);
    EXPECT_GT(report.late_normal_force[leg] / report.late_force_samples, 10.0);
  }
}

/** 2 号原地踏步使用扫参选定的 TROT（0.30 s/50% 支撑）执行行为。 */
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
  // 20 s / 0.30 s ≈ 67 个周期，扣除入场保持后每条腿约 66 次离地。
  expectHealthyMarch(report, 4U, 0.50, 0.30, 60U, 70U);
  EXPECT_LT(static_cast<double>(report.frames_with_three_or_more_swing) /
    static_cast<double>(report.frames), 0.15);
}

/** 方向行走复用 MIT TROT，在零速度命令下也必须原地站稳。 */
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

  expectHealthyMarch(report, 4U, 0.50, 0.30, 60U, 70U);
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
      EXPECT_LT(report.maximum_roll, 0.22);
      EXPECT_LT(report.maximum_pitch, 0.20);
  }
}
