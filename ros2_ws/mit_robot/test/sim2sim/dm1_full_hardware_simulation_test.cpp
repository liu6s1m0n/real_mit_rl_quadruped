#include <atomic>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <thread>
#include <gtest/gtest.h>
#if defined(MYMIT_ENABLE_OPENDOG)
#include <rclcpp/rclcpp.hpp>
#endif

#include "HardwareSimulation.hpp"
#include "hardware/dm1_hardware_driver.hpp"
#include "hardware/hardware_bridge.hpp"
#include "RobotRunner.hpp"
#include "hardware/motor_calibration.hpp"

namespace
{
class Plant
{
public:
  explicit Plant(const HardwareSimulationOptions & settings = {})
  : model(nullptr, mj_deleteModel), data(nullptr, mj_deleteData),
    calibration(dm1_hardware::loadCalibration(MYMIT_ROBOT_TEST_CALIBRATION_PATH))
  {
    char error[1024]{};
    model.reset(mj_loadXML(MYMIT_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)));
    if (!model) {throw std::runtime_error(error);}
    data.reset(mj_makeData(model.get()));
    if (!data) {throw std::runtime_error("cannot allocate simulation data");}
    const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
    if (home < 0) {throw std::runtime_error("home keyframe missing");}
    mj_resetDataKeyframe(model.get(), data.get(), home);
    mj_forward(model.get(), data.get());
    devices = std::make_unique<HardwareSimulation>(model.get(), data.get(), mutex, calibration, settings);
    physics = std::thread([this]() {
      auto next = HardwareSimulation::Clock::now();
      const auto period = std::chrono::duration_cast<HardwareSimulation::Clock::duration>(
        std::chrono::duration<double>(model->opt.timestep));
      try {
        while (!stop.load()) {
          {
            std::lock_guard<std::recursive_mutex> lock(mutex);
            devices->tick(true);
            mj_step(model.get(), data.get());
          }
          next += period;
          if (next < HardwareSimulation::Clock::now()) {next = HardwareSimulation::Clock::now();}
          std::this_thread::sleep_until(next);
        }
      } catch (...) {failure = std::current_exception();}
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
  }
  ~Plant()
  {
    stop.store(true);
    physics.join();
  }
  std::recursive_mutex mutex;
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data;
  HardwareSimulation::Calibration calibration;
  std::unique_ptr<HardwareSimulation> devices;
  std::atomic_bool stop{false};
  std::thread physics;
  std::exception_ptr failure;
};

TEST(Dm1FullHardwareSimulation, RealDriversDecodeRawDevicesAndCalibration)
{
  Plant plant;
  Dm1HardwareDriver driver("sim0", "sim1", plant.devices->imuDevice(), plant.calibration, plant.devices.get());
  ASSERT_TRUE(driver.open());
  dm1_hardware::HardwareSample sample;
  ASSERT_TRUE(driver.read(sample, 1.0));
  EXPECT_TRUE(sample.imu.valid);
  EXPECT_GT(sample.imu.sequence, 100U);  // Real serial baseline collected 200 packets.
  EXPECT_NEAR(sample.imu.gravity_magnitude, 9.81, 0.3);
  dm1_hardware::Dm1MitInterface mit(driver, plant.calibration);
  ASSERT_TRUE(mit.updateFeedback(sample.motors, 1.0));
  const auto joints = mit.jointStates(1.0F);
  std::lock_guard<std::recursive_mutex> lock(plant.mutex);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const char * legs[] = {"FR", "FL", "RR", "RL"};
    const char * axes[] = {"hip", "thigh", "calf"};
    const auto name = std::string(legs[i / 3]) + "_" + axes[i % 3] + "_joint";
    const int joint = mj_name2id(plant.model.get(), mjOBJ_JOINT, name.c_str());
    ASSERT_GE(joint, 0);
    EXPECT_NEAR(joints[i / 3].position[i % 3], plant.data->qpos[plant.model->jnt_qposadr[joint]], 0.002);
    EXPECT_GT(sample.motors[i].sequence, 0U);
    EXPECT_TRUE(sample.motors[i].health_valid);
  }
  EXPECT_FALSE(plant.failure);
  // Unlock before driver's destructor closes the simulated transport.
}

struct Report
{
  int result = -1;
  std::size_t standing = 0, marching = 0, fallback = 0, writes = 0, drops = 0;
  double min_height = 100, peak_torque = 0, max_tracking_error = 0;
  double weak_tracking_error = 0, weak_peak_torque = 0;
  bool stop_returned_to_stand = false, disabled = false;
  double height_estimation_error = 0, roll_estimation_error = 0, pitch_estimation_error = 0;
  std::size_t missing_contacts = 0;
};

Report runFullBridge(HardwareSimulationOptions settings)
{
  Plant plant(settings);
  Dm1HardwareDriver driver("sim0", "sim1", plant.devices->imuDevice(), plant.calibration, plant.devices.get());
  std::atomic_bool stop{false};
  HardwareBridge::Options options;
  options.keyboard_control = true;
  options.torque_limit_nm = settings.torque_limit_nm;
  Report report;
  auto start = HardwareSimulation::Clock::time_point{};
  bool enable_sent = false, stand_sent = false, march_sent = false, stop_sent = false;
  bool ready = false;
  FSM_StateName previous = FSM_StateName::PASSIVE;
  options.command_source = [&]() {
    const auto now = HardwareSimulation::Clock::now();
    if (start == HardwareSimulation::Clock::time_point{}) {start = now;}
    const double elapsed = std::chrono::duration<double>(now - start).count();
    const auto body_height = [&]() {
        std::lock_guard<std::recursive_mutex> lock(plant.mutex);
        const int trunk = mj_name2id(plant.model.get(), mjOBJ_BODY, "trunk");
        return trunk >= 0 ? plant.data->xpos[trunk * 3 + 2] : 0.0;
      };
    std::vector<teleop::OperatorCommand> commands;
    int key = 0;
    if (elapsed > 0.25 && !enable_sent) {enable_sent = true; key = 'U';}
    else if (elapsed > 1.8 && !stand_sent) {stand_sent = true; key = '1';}
    else if (report.standing >= 350 && ready && body_height() >= 0.278 && !march_sent) {
      // 0.28 m 站姿：等机身基本到目标高度再踏步，避免把站起过程计入踏步最低高度。
      march_sent = true;
      key = '2';
    } else if (report.marching >= 700U && !stop_sent) {stop_sent = true; key = ' ';}
    else if (stop_sent && report.stop_returned_to_stand) {key = 27;}
    if (settings.weak_joint >= 0 && report.weak_tracking_error > 0.4 && elapsed > 4) {key = 27;}
    if (elapsed > 18) {stop.store(true);}
    if (key) {commands.push_back(*teleop::decodeKey(key));}
    return commands;
  };
  options.control_observer = [&](const RobotRunner & runner) {
    const auto state = runner.currentStateName();
    ready = runner.locomotionEntryReady();
    if (state == FSM_StateName::BALANCE_STAND) {
      ++report.standing;
      if (stop_sent) {report.stop_returned_to_stand = true;}
      else if (previous == FSM_StateName::LOCOMOTION) {++report.fallback;}
    }
    if (state == FSM_StateName::LOCOMOTION) {
      ++report.marching;
      std::lock_guard<std::recursive_mutex> lock(plant.mutex);
      const int trunk = mj_name2id(plant.model.get(), mjOBJ_BODY, "trunk");
      report.min_height = std::min(report.min_height, plant.data->xpos[trunk * 3 + 2]);
      for (int j = 0; j < plant.model->nv; ++j) {
        report.peak_torque = std::max(report.peak_torque, std::abs(plant.data->qfrc_applied[j]));
      }
    }
    {
      std::lock_guard<std::recursive_mutex> lock(plant.mutex);
      const char * legs[] = {"FR", "FL", "RR", "RL"};
      const char * axes[] = {"hip", "thigh", "calf"};
      if (ready || state == FSM_StateName::LOCOMOTION) {
        const int trunk = mj_name2id(plant.model.get(), mjOBJ_BODY, "trunk");
        const auto * rotation = plant.data->xmat + 9 * trunk;
        const double z = plant.data->xpos[3 * trunk + 2];
        const double roll = std::atan2(rotation[7], rotation[8]);
        const double pitch = std::atan2(-rotation[6], std::hypot(rotation[0], rotation[3]));
        const auto & estimate = runner.stateEstimate();
        report.height_estimation_error = std::max(report.height_estimation_error, std::abs(estimate.position_world.z() - z));
        report.roll_estimation_error = std::max(report.roll_estimation_error, std::abs(estimate.rpy.x() - roll));
        report.pitch_estimation_error = std::max(report.pitch_estimation_error, std::abs(estimate.rpy.y() - pitch));
        std::size_t contacts = 0;
        const int floor = mj_name2id(plant.model.get(), mjOBJ_GEOM, "floor");
        for (const auto * leg : legs) {
          const int foot = mj_name2id(plant.model.get(), mjOBJ_GEOM, leg);
          for (int c = 0; c < plant.data->ncon; ++c) {
            const auto & contact = plant.data->contact[c];
            if ((contact.geom1 == floor && contact.geom2 == foot) ||
              (contact.geom2 == floor && contact.geom1 == foot)) {++contacts; break;}
          }
        }
        if (state == FSM_StateName::LOCOMOTION && contacts != 4) {++report.missing_contacts;}
      }
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        for (std::size_t axis = 0; axis < kJointsPerLeg; ++axis) {
          const auto name = std::string(legs[leg]) + "_" + axes[axis] + "_joint";
          const int joint = mj_name2id(plant.model.get(), mjOBJ_JOINT, name.c_str());
          report.max_tracking_error = std::max(report.max_tracking_error, std::abs(
            runner.jointCommands()[leg].position_desired[axis] -
            plant.data->qpos[plant.model->jnt_qposadr[joint]]));
          if (static_cast<int>(leg * kJointsPerLeg + axis) == settings.weak_joint) {
            report.weak_tracking_error = std::max(report.weak_tracking_error, std::abs(
              runner.jointCommands()[leg].position_desired[axis] -
              plant.data->qpos[plant.model->jnt_qposadr[joint]]));
            report.weak_peak_torque = std::max(report.weak_peak_torque, std::abs(
              plant.data->qfrc_applied[plant.model->jnt_dofadr[joint]]));
          }
        }
      }
    }
    previous = state;
  };
  HardwareBridge bridge(driver, plant.calibration, options);
  report.result = bridge.run(stop);
  {
    std::lock_guard<std::recursive_mutex> lock(plant.mutex);
    report.writes = plant.devices->mitWrites();
    report.drops = plant.devices->droppedWrites();
    report.disabled = !plant.devices->motorsEnabled();
  }
  std::printf("full_hardware result=%d stand=%zu trot=%zu fallback=%zu min_trot_z=%.3f "
    "peak_actual_tau=%.2f max_q_error=%.3f weak_q_error=%.3f weak_actual_tau=%.3f "
    "writes=%zu drops=%zu disabled=%d\n",
    report.result, report.standing, report.marching, report.fallback,
    report.marching ? report.min_height : std::nan(""), report.peak_torque,
    report.max_tracking_error, report.weak_tracking_error, report.weak_peak_torque,
    report.writes, report.drops, report.disabled);
  return report;
}

TEST(Dm1FullHardwareSimulation, CompleteBridgeStandsMarchesAndStops)
{
  const auto report = runFullBridge({});
  EXPECT_EQ(report.result, 0);
  EXPECT_GE(report.standing, 350U);
  EXPECT_GE(report.marching, 700U);
  EXPECT_EQ(report.fallback, 0U);
  EXPECT_GT(report.min_height, 0.27);
  EXPECT_TRUE(report.stop_returned_to_stand);
  EXPECT_TRUE(report.disabled);
  EXPECT_GT(report.writes, 1200U);
}

TEST(Dm1FullHardwareSimulation, MissingCalfOutputExposesTrackingFailure)
{
  HardwareSimulationOptions settings;
  settings.weak_joint = 5;  // FL_calf, implicated in the recorded hardware run.
  settings.output_scale = 0.0;
  const auto report = runFullBridge(settings);
  EXPECT_EQ(report.result, 0);
  EXPECT_GT(report.weak_tracking_error, 0.4);
  EXPECT_LT(report.weak_peak_torque, 0.001);
  EXPECT_EQ(report.marching, 0U);
  EXPECT_GT(report.writes, 1200U);  // Accepted commands alone do not prove execution.
  EXPECT_TRUE(report.disabled);
}

TEST(Dm1FullHardwareSimulation, IntermittentWriteFailuresRetainHardwareHoldBehavior)
{
  HardwareSimulationOptions settings;
  settings.drop_every = 17;
  settings.impairment_after_s = 2.0;  // Keep the real enable confirmation intact.
  const auto report = runFullBridge(settings);
  EXPECT_EQ(report.result, 0);
  EXPECT_GT(report.drops, 50U);
  EXPECT_GE(report.marching, 700U);
  EXPECT_TRUE(report.stop_returned_to_stand);
  EXPECT_TRUE(report.disabled);
}
}  // namespace

#if defined(MYMIT_ENABLE_OPENDOG)
TEST(Dm1FullHardwareSimulation, OpenDogUnfoldsBothRearKneesThroughRawDrivers)
{
  rclcpp::init(0, nullptr);
  {
    Plant plant;
    Dm1HardwareDriver driver("sim0", "sim1", plant.devices->imuDevice(),
      plant.calibration, plant.devices.get());
    HardwareBridge::Options options;
    options.keyboard_control = true;
    options.algorithm_backend = AlgorithmBackendId::OpenDog;
    options.algorithm_config_directory = MYMIT_OPENDOG_TEST_CONFIG;
    auto start = HardwareSimulation::Clock::time_point{};
    bool enable_sent = false, stand_sent = false;
    double height = 0.0;
    double zero_error = -1.0;
    std::array<double, kNumLegs> calf{};
    options.command_source = [&]() {
        const auto now = HardwareSimulation::Clock::now();
        if (start == HardwareSimulation::Clock::time_point{}) {start = now;}
        const double elapsed = std::chrono::duration<double>(now - start).count();
        int key = 0;
        if (elapsed > 0.25 && !enable_sent) {enable_sent = true; key = 'U';}
        else if (elapsed > 1.7 && zero_error < 0.0) {
          std::lock_guard<std::recursive_mutex> lock(plant.mutex);
          zero_error = 0.0;
          const char * legs[] = {"FR", "FL", "RR", "RL"};
          const char * axes[] = {"hip", "thigh", "calf"};
          for (const auto * leg : legs) {
            for (const auto * axis : axes) {
              const auto name = std::string(leg) + "_" + axis + "_joint";
              const int joint = mj_name2id(plant.model.get(), mjOBJ_JOINT, name.c_str());
              zero_error = std::max(zero_error,
                std::abs(plant.data->qpos[plant.model->jnt_qposadr[joint]]));
            }
          }
        }
        else if (elapsed > 1.8 && !stand_sent) {stand_sent = true; key = '1';}
        else if (elapsed > 9.5) {
          key = 27;
          std::lock_guard<std::recursive_mutex> lock(plant.mutex);
          height = plant.data->qpos[2];
          const char * names[] = {"FR_calf_joint", "FL_calf_joint", "RR_calf_joint", "RL_calf_joint"};
          for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
            const int joint = mj_name2id(plant.model.get(), mjOBJ_JOINT, names[leg]);
            calf[leg] = plant.data->qpos[plant.model->jnt_qposadr[joint]];
          }
        }
        std::vector<teleop::OperatorCommand> result;
        if (key) {result.push_back(*teleop::decodeKey(key));}
        return result;
      };
    const std::atomic_bool stop{false};
    HardwareBridge bridge(driver, plant.calibration, options);
    EXPECT_EQ(bridge.run(stop), 0);
    std::cout << "opendog_full_hardware height=" << height << " zero_error=" << zero_error;
    for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
      std::cout << " calf_" << leg << "=" << calf[leg];
    }
    std::cout << std::endl;
    EXPECT_GE(zero_error, 0.0);
    EXPECT_LT(zero_error, double(control_config::hardware::startup_zero_tolerance));
    EXPECT_GT(height, 0.25);
    EXPECT_GT(calf[2], 0.59);
    EXPECT_GT(calf[3], 0.59);
  }
  rclcpp::shutdown();
}
#endif
