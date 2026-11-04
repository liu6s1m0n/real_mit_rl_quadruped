/**
 * @file SimulationBridge.hpp
 * @brief 连接 MuJoCo 渲染/物理循环与 OpenDog 控制循环。
 *
 * 主线程负责 GLFW 和绘制，物理线程负责状态读取、控制计算和 mj_step。
 */
#ifndef DM1_ROBOT_SIMULATION_SIMULATION_BRIDGE_HPP_
#define DM1_ROBOT_SIMULATION_SIMULATION_BRIDGE_HPP_

#include "common/control_config.h"
#include <array>
#include <atomic>
#include <string>

#include "common/robot_types.hpp"
#include "simulation/HardwareSimulation.hpp"

/** 管理 MuJoCo GUI 与物理对象生命周期，并将其接到 OpenDog 算法。 */
class SimulationBridge
{
public:
  /**
   * @brief 绑定 MuJoCo 场景文件。
   * @param scene_path MuJoCo XML 场景文件路径。
   */
  explicit SimulationBridge(std::string scene_path);

  /**
   * @brief 启动 MuJoCo 渲染循环和独立的物理/控制线程。
   * @return 仿真正常退出时返回 0；线程中的异常会在这里重新抛出。
   */
  int run();

  /** 启用硬件在环仿真模式；options.enabled 为 false 时使用直接仿真模式。 */
  void setHardwareSimulation(const HardwareSimulationOptions & options);

  /** 设置两种仿真模式共用的 OpenDog 配置目录（config/opendog）。 */
  void setAlgorithmConfigDirectory(std::string config_directory);

private:
  std::string scene_path_;                                     ///< 要加载的 MuJoCo scene.xml 绝对或相对路径。
  std::atomic<int> pending_motion_command_{-1};                ///< GUI 发出的待消费的一次性运动按钮命令。
  std::atomic_bool pending_disable_command_{false};            ///< 不可被其他 GUI 命令覆盖的失能锁存。
  HardwareSimulationOptions hardware_simulation_;              ///< 硬件在环仿真参数。
  std::string algorithm_config_directory_ = "config/opendog";  ///< OpenDog 配置目录。
};

#endif  // DM1_ROBOT_SIMULATION_SIMULATION_BRIDGE_HPP_
