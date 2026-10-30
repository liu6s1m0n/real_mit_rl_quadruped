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
    input.timestamp = 0.002F;
    ASSERT_TRUE(algorithm.compute(input, output));
    EXPECT_TRUE(algorithm.timing().initial_policy);
    EXPECT_DOUBLE_EQ(algorithm.timing().wbc_ms, 0.0);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      EXPECT_TRUE(output.commands[leg].position_desired.isApprox(input.joints[leg].position));
      EXPECT_TRUE(output.commands[leg].velocity_desired.isZero());
      EXPECT_TRUE(output.commands[leg].torque_feedforward.isZero());
    }
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
      ASSERT_EQ(output.backend, AlgorithmBackendId::OpenDog);
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        const auto & command = output.commands[leg];
        EXPECT_EQ(command.leg, static_cast<LegId>(leg));
        EXPECT_TRUE(command.enabled);
        EXPECT_FLOAT_EQ(command.kp[0], 20.0F);
        EXPECT_FLOAT_EQ(command.kp[1], 20.0F);
        EXPECT_FLOAT_EQ(command.kp[2], 20.0F);
        EXPECT_TRUE(command.kd.isConstant(5.0F));
        EXPECT_TRUE(command.position_desired.allFinite());
        EXPECT_TRUE(command.velocity_desired.allFinite());
        EXPECT_TRUE(command.torque_feedforward.allFinite());
        EXPECT_GT(command.torque_feedforward.norm(), 0.01F);
        EXPECT_EQ(command.sequence, initial_sequence + static_cast<std::uint64_t>(frame));
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

TEST(OpenDogAlgorithm, AlignsStartupGravityWithoutScalingDynamicAcceleration)
{
  rclcpp::init(0, nullptr);
  {
    OpenDogAlgorithm nominal(MYMIT_OPENDOG_TEST_CONFIG);
    OpenDogAlgorithm calibrated(MYMIT_OPENDOG_TEST_CONFIG);
    ASSERT_TRUE(nominal.configure());
    ASSERT_TRUE(calibrated.configure());
    ASSERT_TRUE(nominal.start());
    ASSERT_TRUE(calibrated.start());
    ControlInputSnapshot input;
    input.imu.valid = true;
    input.imu.orientation_world_from_body =
      Eigen::AngleAxisf(0.3F, Vec3<float>::UnitZ()) *
      Eigen::AngleAxisf(-0.03F, Vec3<float>::UnitY()) *
      Eigen::AngleAxisf(-0.05F, Vec3<float>::UnitX());
    input.desired.valid = true;
    input.desired.mode = ControlMode::BalanceStand;
    input.desired.body_position_world.z() = 0.28F;
    for (auto & joint : input.joints) {
      joint.position << 0.0F, -0.282961F, 0.739729F;
    }
    ControlOutput nominal_output, calibrated_output;
    for (int frame = 1; frame <= 400; ++frame) {
      input.timestamp = 0.002 * frame;
      const bool have_baseline = frame <= 300;
      const Vec3<float> motion = frame > 200 && frame <= 300 ?
        Vec3<float>(0.2F, -0.1F, 0.4F) : Vec3<float>::Zero().eval();
      input.imu.acceleration_body = input.imu.orientation_world_from_body.conjugate() *
        (motion + Vec3<float>(0.0F, 0.0F, 9.81F));
      ASSERT_TRUE(nominal.compute(input, nominal_output));
      auto measured = input;
      measured.imu.gravity_valid = have_baseline;
      measured.imu.gravity_magnitude = 10.08F;
      measured.imu.acceleration_body = input.imu.orientation_world_from_body.conjugate() *
        (motion + Vec3<float>(0.0F, 0.0F, have_baseline ? 10.08F : 9.81F));
      ASSERT_TRUE(calibrated.compute(measured, calibrated_output));
      const auto & actual = calibrated_output.estimate;
      EXPECT_LT((actual.acceleration_world - motion).norm(), 1e-5F);
      EXPECT_LT((actual.velocity_world - nominal_output.estimate.velocity_world).norm(), 1e-5F);
      EXPECT_LT((actual.position_world - nominal_output.estimate.position_world).norm(), 1e-5F);
      EXPECT_EQ(actual.gravity_valid, have_baseline);
      EXPECT_FLOAT_EQ(actual.gravity_magnitude, have_baseline ? 10.08F : 9.81F);
      EXPECT_TRUE(actual.acceleration_body.isApprox(measured.imu.acceleration_body));
    }
    nominal.stop();
    calibrated.stop();
  }
  rclcpp::shutdown();
}
