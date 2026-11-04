/*! @file opendog_algorithm_test.cpp
 *  @brief OpenDogAlgorithm 单元测试：配置失败路径和无硬件的站立求解。
 */
#include <array>
#include <cmath>
#include <chrono>
#include <string>
#include <thread>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "control/opendog_algorithm.hpp"

// 没有 ROS 上下文时也能安全构造和析构算法对象（不调用 configure()）。
TEST(OpenDogAlgorithm, CanRemainInactiveWithoutRosContext)
{
  EXPECT_NO_THROW({OpenDogAlgorithm inactive(DM1_OPENDOG_TEST_CONFIG);});
}

// 不连接硬件也能为 DM1 站立求出有限的关节命令。
TEST(OpenDogAlgorithm, SolvesDm1StandingWithoutHardware)
{
  rclcpp::init(0, nullptr);
  {
    OpenDogAlgorithm algorithm(DM1_OPENDOG_TEST_CONFIG);
    ASSERT_TRUE(algorithm.configure());
    ControlInputSnapshot input;
    input.imu.valid = true;
    input.imu.orientation_world_from_body.setIdentity();
    input.imu.acceleration_body << 0.0F, 0.0F, 9.81F;
    input.desired.valid = true;
    input.desired.mode = ControlMode::BalanceStand;
    input.desired.body_position_world.z() = 0.32F;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      input.joints[leg].leg = static_cast<LegId>(leg);
      input.joints[leg].position << 0.0F, -0.485961F, -1.480271F;
    }
    ControlOutput output;
    input.timestamp = 0.002F;
    ASSERT_TRUE(algorithm.compute(input, output));
    // 与上游 LeggedController::starting() 一致：首帧同步求得初始策略后直接走 WBC。
    EXPECT_TRUE(algorithm.timing().initial_policy);
    EXPECT_TRUE(output.valid);
    boost::property_tree::ptree task;
    boost::property_tree::read_info(
      std::string(DM1_OPENDOG_TEST_CONFIG) + "/opendog_task.info", task);
    const float kp = task.get<float>("hardware_output.kp");
    const float kd = task.get<float>("hardware_output.kd");
    const std::array<float, kNumLegs> calf_kp{
      task.get<float>("hardware_output.FR_calf_kp"), task.get<float>("hardware_output.FL_calf_kp"),
      task.get<float>("hardware_output.RR_calf_kp"), task.get<float>("hardware_output.RL_calf_kp")};
    const auto startup = std::chrono::steady_clock::now();
    while (algorithm.timing().initial_policy) {
      ASSERT_LT(std::chrono::steady_clock::now() - startup, std::chrono::seconds(5));
      input.timestamp += 0.002F;
      ASSERT_TRUE(algorithm.compute(input, output));
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto initial_sequence = output.commands[0].sequence;
    for (int frame = 1; frame <= 30; ++frame) {
      input.timestamp += 0.002F;
      ASSERT_TRUE(algorithm.compute(input, output));
      ASSERT_TRUE(output.valid);
      EXPECT_FALSE(algorithm.timing().initial_policy);
      EXPECT_GE(algorithm.timing().compute_ms, algorithm.timing().wbc_ms);
      EXPECT_GT(algorithm.timing().mpc_ms, 0.0);
      EXPECT_GE(algorithm.timing().policy_age_ms, 0.0);
      EXPECT_GT(algorithm.timing().horizon_remaining_ms, 0.0);
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const auto & command = output.commands[leg];
        EXPECT_EQ(command.leg, static_cast<LegId>(leg));
        EXPECT_TRUE(command.enabled);
        EXPECT_FLOAT_EQ(command.kp[0], kp);
        EXPECT_FLOAT_EQ(command.kp[1], kp);
        EXPECT_FLOAT_EQ(command.kp[2], calf_kp[leg]);
        EXPECT_TRUE(command.kd.isConstant(kd));
        EXPECT_TRUE(command.position_desired.allFinite());
        EXPECT_TRUE(command.velocity_desired.allFinite());
        EXPECT_TRUE(command.torque_feedforward.allFinite());
        EXPECT_GT(command.torque_feedforward.norm(), 0.01F);
        EXPECT_EQ(command.sequence, initial_sequence + static_cast<std::uint64_t>(frame));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    algorithm.stop();
    input.timestamp += 0.002F;
    ASSERT_TRUE(algorithm.compute(input, output));
    EXPECT_TRUE(output.valid);
    algorithm.stop();
  }
  rclcpp::shutdown();
}
