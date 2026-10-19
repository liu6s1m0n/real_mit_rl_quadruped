// 模型契约回归测试：dm1_model_contract.yaml 必须与 dm1.xml、dm1.hpp 和
// 控制参数保持一致。任何一处漂移都直接 fail，避免“改了模型但契约没改”。
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include "model/quadruped.hpp"
#include "model/robot_control_parameters.hpp"
#include "model/robots/dm1.hpp"

namespace
{
using ModelPointer = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;

constexpr std::array<const char *, kNumLegs> kLegNames{"FR", "FL", "RR", "RL"};
constexpr std::array<const char *, kJointsPerLeg> kJointNames{"hip", "thigh", "calf"};

ModelPointer loadDm1()
{
  char error[1024]{};
  ModelPointer model(
    mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)),
    &mj_deleteModel);
  if (!model) {throw std::runtime_error(error);}
  return model;
}

YAML::Node loadContract()
{
  return YAML::LoadFile(MYMIT_ROBOT_CONTRACT_PATH);
}

std::string sha256File(const std::string & path)
{
  // coreutils 的 sha256sum 在目标 Linux 环境中始终可用；避免为测试引入
  // OpenSSL 运行时依赖（Conda 的 libcrypto 可能与系统版本冲突）。
  const std::string command = "sha256sum '" + path + "'";
  FILE * pipe = ::popen(command.c_str(), "r");
  if (pipe == nullptr) {throw std::runtime_error("popen failed for " + path);}

  std::array<char, 256> buffer{};
  std::string output;
  while (std::fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    output += buffer.data();
  }
  const int status = ::pclose(pipe);
  if (status != 0) {
    throw std::runtime_error("sha256sum failed for " + path);
  }

  const auto separator = output.find(' ');
  if (separator != 64) {
    throw std::runtime_error("unexpected sha256sum output: " + output);
  }
  return output.substr(0, 64);
}

}  // namespace

TEST(Dm1ContractFile, IdentityAndHashMatch)
{
  const auto contract = loadContract();

  EXPECT_EQ(contract["model"].as<std::string>(), "DM1");
  EXPECT_EQ(contract["contract_version"].as<int>(), 1);
  EXPECT_EQ(
    contract["mjcf"]["file"].as<std::string>(),
    "src/dm1_description/dm1/dm1.xml");
  EXPECT_EQ(
    contract["mjcf"]["sha256"].as<std::string>(),
    sha256File(MYMIT_ROBOT_DM1_XML_PATH));
}

TEST(Dm1ContractFile, JointOrderMatchesMujoco)
{
  const auto contract = loadContract();
  const auto model = loadDm1();

  const auto leg_order = contract["leg_order"].as<std::vector<std::string>>();
  ASSERT_EQ(leg_order.size(), kNumLegs);
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    EXPECT_EQ(leg_order[leg], kLegNames[leg]);
  }

  const auto joint_order_per_leg =
    contract["joint_order_per_leg"].as<std::vector<std::string>>();
  ASSERT_EQ(joint_order_per_leg.size(), kJointsPerLeg);
  for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
    EXPECT_EQ(joint_order_per_leg[joint], kJointNames[joint]);
  }

  const auto joint_order = contract["joint_order"].as<std::vector<std::string>>();
  ASSERT_EQ(joint_order.size(), kNumJoints);

  std::vector<std::string> mujoco_joint_order;
  for (int joint = 0; joint < model->njnt; ++joint) {
    if (model->jnt_type[joint] == mjJNT_FREE) {continue;}
    const char * name = mj_id2name(model.get(), mjOBJ_JOINT, joint);
    ASSERT_NE(name, nullptr);
    mujoco_joint_order.emplace_back(name);
  }
  ASSERT_EQ(mujoco_joint_order.size(), joint_order.size());

  for (std::size_t index = 0; index < joint_order.size(); ++index) {
    const std::size_t leg = index / kJointsPerLeg;
    const std::size_t joint = index % kJointsPerLeg;
    std::string mujoco_name = mujoco_joint_order[index];
    const std::string suffix = "_joint";
    ASSERT_GT(mujoco_name.size(), suffix.size());
    ASSERT_EQ(mujoco_name.compare(mujoco_name.size() - suffix.size(), suffix.size(), suffix), 0);
    mujoco_name.resize(mujoco_name.size() - suffix.size());

    EXPECT_EQ(joint_order[index], mujoco_name);
    EXPECT_EQ(
      joint_order[index],
      std::string(kLegNames[leg]) + "_" + joint_order_per_leg[joint]);
  }
}

TEST(Dm1ContractFile, LimitsAndStandPoseMatchController)
{
  const auto contract = loadContract();
  const auto model = loadDm1();
  const auto controller = robots::dm1::makeModel<double>();
  const auto parameters = makeRobotControlParameters<double>(RobotType::DM1);

  const auto lower =
    contract["joint_lower_limit_rad"].as<std::vector<double>>();
  const auto upper =
    contract["joint_upper_limit_rad"].as<std::vector<double>>();
  const auto zero =
    contract["motor_zero_position_rad"].as<std::vector<double>>();
  const auto offset =
    contract["kinematic_joint_offset_rad"].as<std::vector<double>>();
  ASSERT_EQ(lower.size(), kNumJoints);
  ASSERT_EQ(upper.size(), kNumJoints);
  ASSERT_EQ(zero.size(), kNumJoints);
  ASSERT_EQ(offset.size(), kJointsPerLeg);

  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    const auto & joints =
      controller.leg(static_cast<LegId>(leg)).joints;
    for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
      const std::size_t index = leg * kJointsPerLeg + joint;
      const auto eigen_joint = static_cast<Eigen::Index>(joint);

      EXPECT_NEAR(lower[index], joints.lower_limit[eigen_joint], 1.0e-12);
      EXPECT_NEAR(upper[index], joints.upper_limit[eigen_joint], 1.0e-12);
      EXPECT_NEAR(zero[index], parameters.motor_zero_position[eigen_joint], 1.0e-12);
      EXPECT_NEAR(offset[joint], joints.zero_offset[eigen_joint], 1.0e-12);

      const int joint_id = mj_name2id(
        model.get(), mjOBJ_JOINT,
        (std::string(kLegNames[leg]) + "_" + kJointNames[joint] + "_joint").c_str());
      ASSERT_GE(joint_id, 0);
      ASSERT_NE(model->jnt_limited[joint_id], 0);
      EXPECT_NEAR(model->jnt_range[2 * joint_id], lower[index], 1.0e-12);
      EXPECT_NEAR(model->jnt_range[2 * joint_id + 1], upper[index], 1.0e-12);
    }
  }

  const auto front =
    contract["stand_joint_position_front_rad"].as<std::vector<double>>();
  const auto rear =
    contract["stand_joint_position_rear_rad"].as<std::vector<double>>();
  ASSERT_EQ(front.size(), kJointsPerLeg);
  ASSERT_EQ(rear.size(), kJointsPerLeg);
  for (std::size_t joint = 0; joint < kJointsPerLeg; ++joint) {
    const auto eigen_joint = static_cast<Eigen::Index>(joint);
    EXPECT_NEAR(
      front[joint], controller.leg(LegId::FR).joints.home_position[eigen_joint],
      1.0e-12);
    EXPECT_NEAR(
      front[joint], controller.leg(LegId::FL).joints.home_position[eigen_joint],
      1.0e-12);
    EXPECT_NEAR(
      rear[joint], controller.leg(LegId::RR).joints.home_position[eigen_joint],
      1.0e-12);
    EXPECT_NEAR(
      rear[joint], controller.leg(LegId::RL).joints.home_position[eigen_joint],
      1.0e-12);

    EXPECT_GE(front[joint], lower[joint]);
    EXPECT_LE(front[joint], upper[joint]);
    EXPECT_GE(rear[joint], lower[joint]);
    EXPECT_LE(rear[joint], upper[joint]);
  }
}

TEST(Dm1ContractFile, HeightMassAndControlParametersMatch)
{
  const auto contract = loadContract();
  const auto model = loadDm1();
  const auto controller = robots::dm1::makeModel<double>();
  const auto parameters = makeRobotControlParameters<double>(RobotType::DM1);

  const int trunk = mj_name2id(model.get(), mjOBJ_BODY, "trunk");
  ASSERT_GE(trunk, 0);

  EXPECT_NEAR(
    contract["nominal_body_height_m"].as<double>(),
    controller.nominalBodyHeight(), 1.0e-12);
  EXPECT_NEAR(
    contract["body_mass_kg"].as<double>(),
    controller.bodyInertia().mass, 1.0e-12);
  EXPECT_NEAR(
    contract["body_mass_kg"].as<double>(),
    model->body_mass[trunk], 1.0e-12);

  const auto control = contract["control"];
  EXPECT_NEAR(
    control["minimum_standing_height_m"].as<double>(),
    parameters.minimum_standing_height, 1.0e-12);
  EXPECT_NEAR(
    control["maximum_standing_height_m"].as<double>(),
    parameters.maximum_standing_height, 1.0e-12);
  EXPECT_NEAR(
    control["standing_height_rate_mps"].as<double>(),
    parameters.standing_height_rate, 1.0e-12);
  EXPECT_NEAR(
    control["stand_up_duration_s"].as<double>(),
    parameters.stand_up_duration, 1.0e-12);
  EXPECT_NEAR(
    control["stand_up_thigh_speed_scale"].as<double>(),
    parameters.stand_up_thigh_speed_scale, 1.0e-12);
  EXPECT_NEAR(
    control["physics_timestep_s"].as<double>(),
    model->opt.timestep, 1.0e-12);

  EXPECT_NEAR(
    contract["motor"]["peak_torque_nm"].as<double>(),
    controller.leg(LegId::FR).joints.torque_limit[0], 1.0e-12);
}
