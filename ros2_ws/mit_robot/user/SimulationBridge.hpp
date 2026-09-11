/**
 * @file SimulationBridge.hpp
 * @brief 连接 MuJoCo 渲染/物理循环与 RobotRunner 控制循环。
 *
 * 主线程负责 GLFW 和绘制，物理线程负责状态读取、控制计算和 mj_step。
 */
#ifndef MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
#define MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_

#include <array>
#include <atomic>
#include <string>

#include "controller/frozen_dwaq_policy.hpp"
#include "controller/RlPolicy.hpp"
#include "model/robot_types.hpp"

/** 管理 MuJoCo GUI 与物理对象生命周期，并将其接到 RobotRunner。 */
class SimulationBridge
{
public:
  /**
   * @brief 绑定 MuJoCo 场景文件和机器人型号。
   * @param scene_path MuJoCo XML 场景文件路径。
   * @param robot_type 当前场景对应的机器人型号，用于选择控制参数。
   */
  SimulationBridge(std::string scene_path, RobotType robot_type);

  /**
   * @brief 启动 MuJoCo 渲染循环和独立的物理/控制线程。
   * @return 仿真正常退出时返回 0；线程中的异常会在这里重新抛出。
   */
  int run();

  /**
   * @brief 设置期望站立高度。
   * @param height 机身目标高度，单位 m。
   * @throws std::invalid_argument 高度不是有限值或超出机器人参数范围时抛出。
   */
  void setStandingHeight(float height);

  /**
   * @brief 设置低速前进档速度。
   * @param speed 前进速度，单位 m/s。
   */
  void setSlowWalkingForwardSpeed(float speed);

  /**
   * @brief 设置快速前进档速度。
   * @param speed 前进速度，单位 m/s。
   */
  void setFastWalkingForwardSpeed(float speed);

  /**
   * @brief 设置左右平移档速度的绝对值。
   * @param speed 横向速度大小，单位 m/s。
   */
  void setWalkingLateralSpeed(float speed);

  /**
   * @brief 设置原地旋转档角速度的绝对值。
   * @param yaw_rate 偏航角速度大小，单位 rad/s，正方向为逆时针。
   */
  void setTurningYawRate(float yaw_rate);

  /**
   * @brief 选择行走控制器；MPC 保持原有 Locomotion 控制链，RL 使用选定模型。
   * @param mode 只接受 ControlMode::Locomotion（MPC）或 ControlMode::WalkRl（RL）。
   */
  void setWalkingControllerMode(ControlMode mode);

  /** @brief 选择 RL 模型；默认使用已验证的 model_4210。 */
  void setRlModel(FrozenDwaqModel model);

  /** @brief 读取启动时/下一次行走使用的控制器选择。 */
  ControlMode walkingControllerMode() const noexcept {return walking_mode_;}

  /** @brief 读取当前目标站立高度，单位 m。 */
  float standingHeight() const noexcept {return standing_height_.load();}

  /** @brief 读取低速前进档速度，单位 m/s。 */
  float slowWalkingForwardSpeed() const noexcept {return slow_walking_forward_speed_;}

  /** @brief 读取快速前进档速度，单位 m/s。 */
  float fastWalkingForwardSpeed() const noexcept {return fast_walking_forward_speed_;}

  /** @brief 读取左右平移速度的绝对值，单位 m/s。 */
  float walkingLateralSpeed() const noexcept {return walking_lateral_speed_;}

  /** @brief 读取原地旋转角速度的绝对值，单位 rad/s。 */
  float turningYawRate() const noexcept {return turning_yaw_rate_;}

private:
  std::string scene_path_;  ///< 要加载的 MuJoCo scene.xml 绝对或相对路径。
  RobotType robot_type_;    ///< 与场景匹配的控制器参数型号。
  std::atomic<float> standing_height_{0.39F};  ///< GUI/ROS 传给物理线程的目标机身高度，m。
  std::atomic<int> pending_motion_command_{-1};  ///< GUI 发出的待消费的一次性运动按钮命令。
  std::atomic_bool pending_disable_command_{false};  ///< 不可被其他 GUI 命令覆盖的失能锁存。
  float slow_walking_forward_speed_ = 0.18F;  ///< 低速前进档速度，m/s。
  float fast_walking_forward_speed_ = 0.36F;  ///< 快速前进档速度，m/s。
  float walking_backward_speed_ = 0.30F;      ///< 后退档速度绝对值，m/s。
  float walking_lateral_speed_ = 0.25F;       ///< 左右平移档速度绝对值，m/s。
  float turning_yaw_rate_ = 0.35F;            ///< 原地自转角速度绝对值，rad/s。
  double standing_height_slider_ = 0.39;      ///< MuJoCo UI 滑块使用的双精度高度缓存，m。
  ControlMode walking_mode_ = ControlMode::Locomotion;  ///< MPC 或 frozen RL。
  FrozenDwaqModel rl_model_ = FrozenDwaqModel::Model4210;
  RlPolicyPtr rl_policy_;                    ///< 仅 RL 模式注入，MPC 不触碰。
};

#endif  // MYMIT_ROBOT_USER_SIMULATION_BRIDGE_HPP_
