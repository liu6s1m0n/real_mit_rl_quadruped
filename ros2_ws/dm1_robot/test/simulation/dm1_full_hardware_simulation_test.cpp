/*! @file dm1_full_hardware_simulation_test.cpp
 *  @brief 硬件在环全链路测试：真实驱动读取 HardwareSimulation 模拟的原始设备数据。
 */
#include <array>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "simulation/HardwareSimulation.hpp"
#include "hardware/dm1_hardware_driver.hpp"
#include "hardware/hardware_bridge.hpp"
#include "hardware/motor_calibration.hpp"

namespace
{
/// 测试用的 MuJoCo 场景 + HardwareSimulation + 真机驱动组合。
class Plant
{
public:
  explicit Plant(const HardwareSimulationOptions & settings = {})
  : model(nullptr, mj_deleteModel), data(nullptr, mj_deleteData),
    calibration(dm1_hardware::loadCalibration(DM1_ROBOT_TEST_CALIBRATION_PATH))
  {
    char error[1024]{};
    model.reset(mj_loadXML(DM1_ROBOT_TEST_SCENE_PATH, nullptr, error, sizeof(error)));
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

// 真机驱动能解码模拟设备的原始帧并正确应用标定。
TEST(Dm1FullHardwareSimulation, RealDriversDecodeRawDevicesAndCalibration)
{
  Plant plant;
  Dm1HardwareDriver driver("sim0", "sim1", plant.devices->imuDevice(), plant.calibration, plant.devices.get());
  ASSERT_TRUE(driver.open());
  dm1_hardware::HardwareSample sample;
  ASSERT_TRUE(driver.read(sample, 1.0));
  EXPECT_TRUE(sample.imu.valid);
  EXPECT_GT(sample.imu.sequence, 100U);  // 真实串口路径的启动基准采集了 200 帧。
  EXPECT_NEAR(sample.imu.gravity_magnitude, 9.81, 0.3);
  dm1_hardware::Dm1MitInterface mit(plant.calibration);
  ASSERT_TRUE(mit.updateFeedback(sample.motors));
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
  // 在驱动析构关闭模拟传输层之前先解锁。
}

}  // namespace

// 经过完整真机驱动链路，OpenDog 站立时两条后腿膝关节都能展开。
TEST(Dm1FullHardwareSimulation, OpenDogUnfoldsBothRearKneesThroughRawDrivers)
{
  rclcpp::init(0, nullptr);
  {
    Plant plant;
    Dm1HardwareDriver driver("sim0", "sim1", plant.devices->imuDevice(),
      plant.calibration, plant.devices.get());
    HardwareBridge::Options options;
    options.keyboard_control = true;
    options.algorithm_config_directory = DM1_OPENDOG_TEST_CONFIG;
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
          // 关节初始化插值到趴卧姿态，而不是电机零位。
          const double prone[] = {
            control_config::pose::prone_hip, control_config::pose::prone_thigh,
            control_config::pose::prone_calf};
          for (const auto * leg : legs) {
            for (std::size_t axis = 0; axis < kJointsPerLeg; ++axis) {
              const auto name = std::string(leg) + "_" + axes[axis] + "_joint";
              const int joint = mj_name2id(plant.model.get(), mjOBJ_JOINT, name.c_str());
              zero_error = std::max(zero_error,
                std::abs(plant.data->qpos[plant.model->jnt_qposadr[joint]] - prone[axis]));
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
    // 趴卧初始化结束时每个关节应在趴卧姿态 0.05 rad 以内。
    EXPECT_LT(zero_error, 0.05);
    EXPECT_GT(height, 0.25);
    EXPECT_GT(calf[2], -1.63);
    EXPECT_GT(calf[3], -1.63);
  }
  rclcpp::shutdown();
}
