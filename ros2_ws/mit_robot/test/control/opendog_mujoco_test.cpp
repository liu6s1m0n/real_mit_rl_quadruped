#include <memory>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include "control/opendog_algorithm.hpp"
#include "sensor/imu.hpp"
#include "sensor/leg.hpp"

namespace
{
struct ModelDeleter {void operator()(mjModel * model) const noexcept {mj_deleteModel(model);}};
struct DataDeleter {void operator()(mjData * data) const noexcept {mj_deleteData(data);}};
}  // namespace

TEST(OpenDogMuJoCo, UsesCurrentDm1SceneFeedback)
{
  char error[1024]{};
  std::unique_ptr<mjModel, ModelDeleter> model(
    mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)));
  ASSERT_NE(model, nullptr) << error;
  std::unique_ptr<mjData, DataDeleter> data(mj_makeData(model.get()));
  ASSERT_NE(data, nullptr);
  const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
  ASSERT_GE(home, 0);
  mj_resetDataKeyframe(model.get(), data.get(), home);
  mj_forward(model.get(), data.get());

  rclcpp::init(0, nullptr);
  {
    SimImu imu(model.get(), data.get());
    std::array<std::unique_ptr<SimLeg>, kNumLegs> legs;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      legs[leg] = std::make_unique<SimLeg>(
        model.get(), data.get(), static_cast<LegId>(leg));
    }
    OpenDogAlgorithm algorithm(MYMIT_OPENDOG_TEST_CONFIG, model->opt.timestep);
    ASSERT_TRUE(algorithm.configure());
    ASSERT_TRUE(algorithm.start());
    ControlInputSnapshot input;
    input.imu = imu.read();
    ASSERT_TRUE(input.imu.valid);
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      input.joints[leg] = legs[leg]->read();
      ASSERT_TRUE(input.joints[leg].valid);
    }
    input.desired.valid = true;
    input.desired.mode = ControlMode::StandUp;
    input.desired.body_position_world.z() = 0.28F;
    input.timestamp = static_cast<float>(data->time);
    ControlOutput output;
    ASSERT_TRUE(algorithm.compute(input, output));
    ASSERT_TRUE(output.valid);
    EXPECT_EQ(output.backend, AlgorithmBackendId::OpenDog);
    for (const auto & command : output.commands) {
      EXPECT_TRUE(command.enabled);
      EXPECT_TRUE(command.position_desired.allFinite());
      EXPECT_TRUE(command.velocity_desired.allFinite());
      EXPECT_TRUE(command.torque_feedforward.allFinite());
    }
    algorithm.stop();
  }
  rclcpp::shutdown();
}
