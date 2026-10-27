#include <cmath>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "control/opendog_algorithm.hpp"

TEST(OpenDogAlgorithm, CanRemainInactiveWithoutRosContext)
{
  EXPECT_NO_THROW({OpenDogAlgorithm inactive(MYMIT_OPENDOG_TEST_CONFIG);});
}

TEST(OpenDogAlgorithm, SolvesDm1StandingWithoutHardware)
{
  rclcpp::init(0, nullptr);
  {
    OpenDogAlgorithm algorithm(MYMIT_OPENDOG_TEST_CONFIG);
    ASSERT_TRUE(algorithm.configure());
    ASSERT_TRUE(algorithm.start());
    ControlInputSnapshot input;
    input.imu.valid = true;
    input.imu.orientation_world_from_body.setIdentity();
    input.imu.acceleration_body << 0.0F, 0.0F, 9.81F;
    input.desired.valid = true;
    input.desired.mode = ControlMode::BalanceStand;
    input.desired.body_position_world.z() = 0.28F;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      input.joints[leg].leg = static_cast<LegId>(leg);
      input.joints[leg].position << 0.0F, -0.282961F, 0.739729F;
    }
    ControlOutput output;
    for (int frame = 1; frame <= 30; ++frame) {
      input.timestamp = 0.002F * frame;
      ASSERT_TRUE(algorithm.compute(input, output));
      ASSERT_TRUE(output.valid);
      EXPECT_EQ(algorithm.timing().initial_policy, frame == 1);
      EXPECT_GE(algorithm.timing().compute_ms, algorithm.timing().wbc_ms);
      EXPECT_GT(algorithm.timing().mpc_ms, 0.0);
      EXPECT_GE(algorithm.timing().policy_age_ms, 0.0);
      EXPECT_GT(algorithm.timing().horizon_remaining_ms, 0.0);
      ASSERT_EQ(output.backend, AlgorithmBackendId::OpenDog);
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const auto & command = output.commands[leg];
        EXPECT_EQ(command.leg, static_cast<LegId>(leg));
        EXPECT_TRUE(command.enabled);
        EXPECT_FLOAT_EQ(command.kp[0], 10.0F);
        EXPECT_FLOAT_EQ(command.kp[1], 10.0F);
        EXPECT_FLOAT_EQ(command.kp[2], leg >= 2 ? 18.0F : 10.0F);
        EXPECT_TRUE(command.kd.isConstant(2.0F));
        EXPECT_TRUE(command.position_desired.allFinite());
        EXPECT_TRUE(command.velocity_desired.allFinite());
        EXPECT_TRUE(command.torque_feedforward.allFinite());
        EXPECT_GT(command.torque_feedforward.norm(), 0.01F);
        EXPECT_EQ(command.sequence, static_cast<std::uint64_t>(frame));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    algorithm.stop();
    ASSERT_TRUE(algorithm.start());
    input.timestamp += 0.002F;
    ASSERT_TRUE(algorithm.compute(input, output));
    EXPECT_TRUE(output.valid);
    algorithm.stop();
  }
  rclcpp::shutdown();
}
