#include <memory>
#include <chrono>
#include <fstream>
#include <cstdlib>
#include <thread>
#include <algorithm>
#include "SimulationActuatorWriter.hpp"

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

// Runs the actual DM1 physics/output path, including both finite diagonals.
// Timing is observational: no admission gate, timeout action, or solver tuning.
TEST(OpenDogMuJoCo, SingleStepsAndMeasuresPolicyFreshness)
{
  char error[1024]{};
  std::unique_ptr<mjModel, ModelDeleter> model(
    mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)));
  ASSERT_NE(model, nullptr) << error;
  std::unique_ptr<mjData, DataDeleter> data(mj_makeData(model.get()));
  mj_resetDataKeyframe(model.get(), data.get(), mj_name2id(model.get(), mjOBJ_KEY, "home"));
  mj_forward(model.get(), data.get());
  rclcpp::init(0, nullptr);
  {
    SimImu imu(model.get(), data.get());
    std::array<std::unique_ptr<SimLeg>, kNumLegs> legs;
    const std::array<const char *, 4> feet{{"FR_foot", "FL_foot", "RR_foot", "RL_foot"}};
    std::array<int, 4> foot_ids;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      legs[leg] = std::make_unique<SimLeg>(model.get(), data.get(), static_cast<LegId>(leg));
      foot_ids[leg] = mj_name2id(model.get(), mjOBJ_BODY, feet[leg]);
      ASSERT_GE(foot_ids[leg], 0);
    }
    OpenDogAlgorithm algorithm(MYMIT_OPENDOG_TEST_CONFIG, model->opt.timestep);
    ASSERT_TRUE(algorithm.configure());
    ASSERT_TRUE(algorithm.start());
    SimulationActuatorWriter writer(model.get());
    ControlInputSnapshot input;
    input.desired.valid = true;
    input.desired.mode = ControlMode::StandUp;
    input.desired.body_position_world.z() = 0.28F;
    ControlOutput output;
    const char * csv_path = std::getenv("OPENDOG_TIMING_CSV");
    std::ofstream csv(csv_path ? csv_path : "/tmp/opendog_freshness.csv");
    ASSERT_TRUE(csv.good());
    csv << "time,stage,compute_ms,wbc_ms,mpc_ms,policy_age_ms,horizon_ms,wall_compute_ms,wall_elapsed_s,mode,z,FR_z,FL_z,RR_z,RL_z\n";
    using Clock = std::chrono::steady_clock;
    auto deadline = Clock::now();
    const auto wall_start = deadline;
    const int base_body = mj_name2id(model.get(), mjOBJ_BODY, "trunk");
    ASSERT_GE(base_body, 0);
    bool first_diagonal_seen = false, second_diagonal_seen = false, repeat_seen = false;
    std::array<double, 4> first_heights{}, second_heights{};
    for (int frame = 0; frame < 7000; ++frame) {
      const double time = data->time;
      int stage = time < 4.0 ? 0 : time < 6.5 ? 1 : time < 9.0 ? 2 : time < 11.0 ? 3 : 4;
      if (stage == 1 || stage == 2 || stage == 4) {
        algorithm.setMotionRequest(stage == 2 ? teleop::Motion::TwoLegStandReverse :
          teleop::Motion::TwoLegStand, stage == 4 ? 4 : stage);
        input.desired.mode = ControlMode::Locomotion;
      } else if (stage == 3) {
        algorithm.setMotionRequest(teleop::Motion::MarchInPlace, 3);
        input.desired.mode = ControlMode::Locomotion;
      }
      input.imu = imu.read();
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {input.joints[leg] = legs[leg]->read();}
      input.timestamp = float(time);
      const auto begin = Clock::now();
      ASSERT_TRUE(algorithm.compute(input, output));
      const auto end = Clock::now();
      const auto & timing = algorithm.timing();
      if (time > 3.5) {
        ASSERT_GT(data->qpos[2], 0.18) << "time=" << time;
        ASSERT_GT(data->xmat[9 * base_body + 8], 0.7) << "time=" << time;
      }
      if (time > 5.5 && time < 6.5) {EXPECT_EQ(timing.planned_mode, 15U);}
      if (time > 8.0 && time < 9.0) {EXPECT_EQ(timing.planned_mode, 15U);}
      first_diagonal_seen |= stage == 1 && timing.planned_mode == 6;
      second_diagonal_seen |= stage == 2 && timing.planned_mode == 9;
      repeat_seen |= time > 12.1 && timing.planned_mode == 6;
      if (time > 12.7) {EXPECT_EQ(timing.planned_mode, 15U);}
      for (int leg = 0; leg < 4; ++leg) {
        const double height = data->xpos[3 * foot_ids[leg] + 2];
        if (stage == 1 && timing.planned_mode == 6) {first_heights[leg] = std::max(first_heights[leg], height);}
        if (stage == 2 && timing.planned_mode == 9) {second_heights[leg] = std::max(second_heights[leg], height);}
        ASSERT_TRUE(output.commands[leg].torque_feedforward.allFinite());
        EXPECT_FLOAT_EQ(output.commands[leg].timestamp, input.timestamp);
      }
      csv << time << ',' << stage << ',' << timing.compute_ms << ',' << timing.wbc_ms << ','
          << timing.mpc_ms << ',' << timing.policy_age_ms << ',' << timing.horizon_remaining_ms << ','
          << std::chrono::duration<double, std::milli>(end - begin).count() << ','
          << std::chrono::duration<double>(end - wall_start).count() << ','
          << timing.planned_mode << ',' << data->qpos[2];
      for (int leg = 0; leg < 4; ++leg) {csv << ',' << data->xpos[3 * foot_ids[leg] + 2];}
      csv << '\n';
      writer.write(output, data.get());
      mj_step(model.get(), data.get());
      deadline += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(model->opt.timestep));
      if (end - deadline > std::chrono::milliseconds(8)) {deadline = end;}
      std::this_thread::sleep_until(deadline);
    }
    EXPECT_TRUE(first_diagonal_seen);
    EXPECT_TRUE(second_diagonal_seen);
    EXPECT_TRUE(repeat_seen);
    EXPECT_GT(std::min(first_heights[1], first_heights[2]) -
      std::max(first_heights[0], first_heights[3]), 0.015);
    EXPECT_GT(std::min(second_heights[0], second_heights[3]) -
      std::max(second_heights[1], second_heights[2]), 0.015);
    algorithm.stop();
  }
  rclcpp::shutdown();
}

// Verify all four knees unfold from the actual DM1 prone home, rather than
// accepting only a lifted front body or a finite WBC command.
TEST(OpenDogMuJoCo, UnfoldsBothRearKneesFromProneHome)
{
  char error[1024]{};
  std::unique_ptr<mjModel, ModelDeleter> model(
    mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)));
  ASSERT_NE(model, nullptr) << error;
  std::unique_ptr<mjData, DataDeleter> data(mj_makeData(model.get()));
  mj_resetDataKeyframe(model.get(), data.get(), mj_name2id(model.get(), mjOBJ_KEY, "home"));
  mj_forward(model.get(), data.get());
  rclcpp::init(0, nullptr);
  {
    SimImu imu(model.get(), data.get());
    std::array<std::unique_ptr<SimLeg>, kNumLegs> legs;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      legs[leg] = std::make_unique<SimLeg>(model.get(), data.get(), static_cast<LegId>(leg));
    }
    const char * config = std::getenv("OPENDOG_STAND_TEST_CONFIG");
    OpenDogAlgorithm algorithm(config ? config : MYMIT_OPENDOG_TEST_CONFIG, model->opt.timestep);
    ASSERT_TRUE(algorithm.configure());
    ASSERT_TRUE(algorithm.start());
    SimulationActuatorWriter writer(model.get());
    ControlInputSnapshot input;
    input.desired.valid = true;
    input.desired.body_position_world.z() = 0.28F;
    ControlOutput output;
    auto deadline = std::chrono::steady_clock::now();
    for (int frame = 0; frame < 4000; ++frame) {
      input.timestamp = static_cast<float>(data->time);
      input.imu = imu.read();
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {input.joints[leg] = legs[leg]->read();}
      input.desired.mode = data->time < 1.0 ? ControlMode::Passive : ControlMode::StandUp;
      ASSERT_TRUE(algorithm.compute(input, output));
      writer.write(output, data.get());
      mj_step(model.get(), data.get());
      deadline += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(model->opt.timestep));
      if (std::chrono::steady_clock::now() - deadline > std::chrono::milliseconds(8)) {
        deadline = std::chrono::steady_clock::now();
      }
      std::this_thread::sleep_until(deadline);
    }
    std::cout << "stand_result height=" << data->qpos[2] << " pitch=" << output.estimate.rpy.y();
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      const auto joint = legs[leg]->read();
      std::cout << " leg=" << leg << " calf=" << joint.position[2]
                << " desired=" << output.commands[leg].position_desired[2]
                << " ff=" << output.commands[leg].torque_feedforward[2];
    }
    std::cout << std::endl;
    EXPECT_GT(data->qpos[2], 0.25);
    for (std::size_t leg : {2U, 3U}) {
      EXPECT_GT(legs[leg]->read().position[2], 0.59F) << "rear leg=" << leg;
    }
    algorithm.stop();
  }
  rclcpp::shutdown();
}
