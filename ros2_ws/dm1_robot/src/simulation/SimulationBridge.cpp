/**
 * @file SimulationBridge.cpp
 * @brief MuJoCo 仿真适配层的实现。
 *
 * 本文件连接 MuJoCo 的渲染/UI 线程、物理线程和机器人控制器：
 * UI 线程产生用户命令，物理线程读取机器人状态并调用 OpenDog 算法，
 * 最后将关节力矩写回 MuJoCo 并推进仿真时间。
 */
#include "simulation/SimulationBridge.hpp"
#include "simulation/SimulationActuatorWriter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <glfw_adapter.h>
#include <mujoco/mujoco.h>
#include <simulate.h>

#include <dlfcn.h>
#include <rclcpp/rclcpp.hpp>

#include "teleop/keyboard_teleop.hpp"
#include "teleop/operator_command.hpp"
#include "hardware/hardware_bridge.hpp"
#include "hardware/dm1_hardware_driver.hpp"
#include "hardware/motor_calibration.hpp"
#include "control/opendog_algorithm.hpp"
#include "sensor/imu.hpp"
#include "sensor/leg.hpp"

namespace mj = ::mujoco;

namespace
{

/**
 * @brief Feed MuJoCo window key events into the same command queue as TTY keys.
 *
 * The previous implementation only read stdin. Once the MuJoCo window had
 * focus, W/A/S/D and the enable/stand keys never reached the controller.
 */
class TeleopGlfwAdapter final : public mj::GlfwAdapter
{
public:
  explicit TeleopGlfwAdapter(teleop::KeyboardTeleop * keyboard, bool hardware_mode = false)
  : keyboard_(keyboard), hardware_mode_(hardware_mode) {}

protected:
  void OnKey(int key, int scancode, int action) override
  {
    (void)scancode;
    // GUI 运动命令是锁存式的。按键重复事件不能充当隐式心跳；
    // 空格才是显式的停止命令。
    int decoded = 0;
    switch (key) {
      case GLFW_KEY_U: decoded = 'U'; break;
      case GLFW_KEY_0: decoded = '0'; break;
      case GLFW_KEY_1: decoded = '1'; break;
      case GLFW_KEY_2: decoded = '2'; break;
      case GLFW_KEY_3: decoded = '3'; break;
      case GLFW_KEY_4: decoded = '4'; break;
      case GLFW_KEY_W: decoded = 'W'; break;
      case GLFW_KEY_S: decoded = 'S'; break;
      case GLFW_KEY_A: decoded = 'A'; break;
      case GLFW_KEY_D: decoded = 'D'; break;
      case GLFW_KEY_Q: decoded = 'Q'; break;
      case GLFW_KEY_E: decoded = 'E'; break;
      case GLFW_KEY_H: decoded = 'H'; break;
      case GLFW_KEY_P: decoded = 'P'; break;
      case GLFW_KEY_SPACE: decoded = ' '; break;
      case GLFW_KEY_ESCAPE: decoded = 27; break;
      default: break;
    }
    // MuJoCo 也把空格绑定为暂停。操作员按键只能送到真正的桥接层，
    // 否则按“停止”会让整个控制器重启。
    if (!hardware_mode_ || decoded == 0) {mj::GlfwAdapter::OnKey(key, scancode, action);}
    if (decoded != 0 && keyboard_ != nullptr && action == GLFW_PRESS) {keyboard_->injectKey(decoded);}
  }

private:
  teleop::KeyboardTeleop * keyboard_ = nullptr;
  bool hardware_mode_ = false;
};

/** MuJoCo UI 添加函数的原始函数指针类型。 */
using MjuiAddFunction = void (*)(mjUI *, const mjuiDef *);
/** MuJoCo UI 事件处理函数的原始函数指针类型。 */
using MjuiEventFunction = mjuiItem * (*)(mjUI *, mjuiState *, const mjrContext *);

// 这些指针在 SimulationBridge::run() 启动 RenderLoop 期间有效。
// C 风格的 MuJoCo 回调通过它们访问主对象中的共享状态。
std::atomic<int> * g_pending_motion_command = nullptr;   ///< 指向 SimulationBridge 的待处理按钮命令。
std::atomic_bool * g_pending_disable_command = nullptr;  ///< 指向 SimulationBridge 的失能锁存。
bool g_hardware_simulation = false;                      ///< 当前是否为硬件在环仿真模式。

/// GUI 按钮对应的一次性命令编号。
enum MotionCommand : int
{
  kNoMotionCommand = -1,        ///< 没有待处理命令。
  kStand = 0,                   ///< 站立。
  kForwardSlow = 1,             ///< 慢速前进。
  kForwardFast = 2,             ///< 快速前进。
  kBackward = 3,                ///< 后退。
  kLeft = 4,                    ///< 左移。
  kRight = 5,                   ///< 右移。
  kRotateCounterClockwise = 6,  ///< 左转。
  kRotateClockwise = 7,         ///< 右转。
  kEnableMotors = 8,            ///< 使能电机。
  kDisableMotors = 9,           ///< 失能电机。
  kStop = 10,                   ///< 停止运动。
  kProneDown = 11,              ///< 趴下。
  /// 原地对角小跑（OpenDog 连续踏步）。
  kMarchInPlace = 12,
  kTwoLegStand = 13             ///< LF/RH 对角单步。
};

/// 把 GUI 按钮名称映射为命令编号；未知按钮返回 kNoMotionCommand。
int commandForButton(const char * name)
{
  if (name == nullptr) {return kNoMotionCommand;}
  if (std::strcmp(name, "Stand up") == 0) {return kStand;}
  if (std::strcmp(name, "Forward slow") == 0) {return kForwardSlow;}
  if (std::strcmp(name, "Forward fast") == 0) {return kForwardFast;}
  if (std::strcmp(name, "Backward") == 0) {return kBackward;}
  if (std::strcmp(name, "Left") == 0) {return kLeft;}
  if (std::strcmp(name, "Right") == 0) {return kRight;}
  if (std::strcmp(name, "Rotate CCW") == 0) {return kRotateCounterClockwise;}
  if (std::strcmp(name, "Rotate CW") == 0) {return kRotateClockwise;}
  if (std::strcmp(name, "Enable motors") == 0) {return kEnableMotors;}
  if (std::strcmp(name, "Disable motors") == 0) {return kDisableMotors;}
  if (std::strcmp(name, "Stop") == 0) {return kStop;}
  if (std::strcmp(name, "Prone down") == 0) {return kProneDown;}
  if (std::strcmp(name, "March in place TROT") == 0) {return kMarchInPlace;}
  if (std::strcmp(name, "Two-leg stand test") == 0) {return kTwoLegStand;}
  return kNoMotionCommand;
}

/**
 * @brief 获取 MuJoCo 原始的 mjui_add 函数。
 *
 * 当前文件导出了同名包装函数，直接调用 mjui_add 会递归进入包装函数；
 * RTLD_NEXT 用于跳过当前符号并找到 MuJoCo 原本的实现。
 */
MjuiAddFunction originalMjuiAdd()
{
  // 取得 MuJoCo 原始函数，避免下面的同名扩展再次调用自身而递归。
  static const MjuiAddFunction function = []() {
      void * symbol = ::dlsym(RTLD_NEXT, "mjui_add");
      MjuiAddFunction result = nullptr;
      static_assert(sizeof(result) == sizeof(symbol));
      std::memcpy(&result, &symbol, sizeof(result));
      return result;
    }();
  return function;
}

/** @brief 获取 MuJoCo 原始的 mjui_event 函数，避免包装函数递归调用自身。 */
MjuiEventFunction originalMjuiEvent()
{
  static const MjuiEventFunction function = []() {
      void * symbol = ::dlsym(RTLD_NEXT, "mjui_event");
      MjuiEventFunction result = nullptr;
      static_assert(sizeof(result) == sizeof(symbol));
      std::memcpy(&result, &symbol, sizeof(result));
      return result;
    }();
  return function;
}

}  // namespace

/**
 * @brief 包装 MuJoCo 的 UI 创建函数，追加机器人控制项。
 *
 * Simulate 在 RenderLoop 内部调用该函数。这里先调用 MuJoCo 原始实现，
 * 创建标准 Simulation 区域，再在同一个渲染线程追加自定义控件。
 */
extern "C" void mjui_add(mjUI * ui, const mjuiDef * definition)
{
  // 保留 MuJoCo 自带 UI 的创建行为。
  const MjuiAddFunction add = originalMjuiAdd();
  if (add == nullptr) {
    std::fprintf(stderr, "unable to resolve MuJoCo mjui_add\n");
    std::abort();
  }
  add(ui, definition);

  // 如果回调发生在共享指针尚未初始化或已经清空之后，只显示原生 UI。
  if (g_pending_motion_command == nullptr || g_pending_disable_command == nullptr ||
    definition == nullptr ||
    definition[0].type != mjITEM_SECTION ||
    definition[0].name == nullptr ||
    std::strcmp(definition[0].name, "Simulation") != 0)
  {
    return;
  }

  // mjuiDef 数组必须以 mjITEM_END 结束。
  mjuiDef controls[] = {
    {mjITEM_SEPARATOR, "Robot controller", 1, nullptr, "", 0},
    {mjITEM_BUTTON, "Forward slow", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Forward fast", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Backward", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Left", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Right", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Rotate CCW", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Rotate CW", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Stand up", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Enable motors", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Disable motors", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Stop", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Prone down", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "March in place TROT", 2, nullptr, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}
  };
  if (g_hardware_simulation) {std::strcpy(controls[0].name, "Hardware controller");}
  add(ui, controls);
}

/**
 * @brief 包装 MuJoCo UI 事件，并转发一次性动作请求。
 *
 * MuJoCo 原函数负责处理鼠标点击和控件状态。本函数只检查返回的控件，
 * 通过 atomic 命令通知物理线程；普通按钮用 exchange(kNoMotionCommand)
 * 消费一次性请求，失能按钮另有不可覆盖的锁存位。
 */
extern "C" mjuiItem * mjui_event(
  mjUI * ui, mjuiState * state, const mjrContext * context)
{
  const MjuiEventFunction event = originalMjuiEvent();
  if (event == nullptr) {
    std::fprintf(stderr, "unable to resolve MuJoCo mjui_event\n");
    std::abort();
  }
  // 先执行原生事件处理，再分析发生变化的控件。
  mjuiItem * changed = event(ui, state, context);
  if (changed != nullptr && changed->type == mjITEM_BUTTON &&
    g_pending_motion_command != nullptr && g_pending_disable_command != nullptr)
  {
    const int command = commandForButton(changed->name);
    if (command == kDisableMotors) {
      g_pending_disable_command->store(true);
    } else if (command != kNoMotionCommand) {
      g_pending_motion_command->store(command);
    }
  }
  return changed;
}

namespace
{

/**
 * @brief 直接仿真模式的物理/控制循环。
 *
 * 每个物理步：检测 GUI 复位 → 收集 GUI/键盘命令并仲裁 → 更新期望状态 →
 * 读取仿真传感器运行 OpenDog 算法 → 写入关节力矩并 mj_step → 按仿真步长限速。
 * 直接模式不经过 OpenDogRobotRunner，因此没有趴卧初始化和 P 趴下流程。
 */
void runOpenDogPhysics(
  mj::Simulate & simulation, mjModel * model, mjData * data,
  const SimulationActuatorWriter & actuator_writer,
  const std::string & algorithm_config_directory,
  std::atomic<int> & pending_motion_command, std::atomic_bool & pending_disable_command,
  teleop::KeyboardTeleop & keyboard)
{
  OpenDogAlgorithm algorithm(algorithm_config_directory, model->opt.timestep);
  if (!algorithm.configure()) {
    throw std::runtime_error("OpenDog MuJoCo backend initialization failed");
  }
  // 与真机 runner 一致：站高取 opendog_reference.info 的 comHeight。
  const float standing_height = algorithm.commandSettings().standing_height;
  SimImu sim_imu(model, data);
  std::array<std::unique_ptr<SimLeg>, kNumLegs> sim_legs;
  for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
    sim_legs[leg] = std::make_unique<SimLeg>(model, data, static_cast<LegId>(leg));
  }
  teleop::OperatorCommandArbiter arbiter;
  DesiredState<float> desired;
  desired.valid = true;
  desired.body_position_world.z() = standing_height;
  ControlOutput output;
  std::uint64_t motion_sequence = 0;
  bool control_active = false;
  bool was_enabled = false;
  bool controller_ready = false;
  std::size_t consecutive_failures = 0;
  double previous_simulation_time = data->time;
  bool keyboard_fault_reported = false;
  using PhysicsClock = std::chrono::steady_clock;
  const auto physics_period = std::chrono::duration_cast<PhysicsClock::duration>(
    std::chrono::duration<double>(model->opt.timestep));
  auto next_physics_deadline = PhysicsClock::now();
  bool was_running = false;

  std::printf(
    "MuJoCo DM1 OpenDog control started: dt=%.4f s, default mode=OpenDog WBC+NMPC\n",
    model->opt.timestep);

  while (!simulation.exitrequest.load()) {
    std::unique_lock<std::recursive_mutex> lock(simulation.mtx);
    const bool simulation_running = simulation.run != 0;
    // 仿真时间回退说明 GUI 执行了复位：复位算法和期望状态，并锁定电机。
    if (data->time + 0.5 * model->opt.timestep < previous_simulation_time) {
      mj_forward(model, data);
      algorithm.reset();
      desired = DesiredState<float>{};
      desired.valid = true;
      desired.body_position_world.z() = standing_height;
      output = ControlOutput{};
      control_active = false;
      was_enabled = false;
      controller_ready = false;
      consecutive_failures = 0;
      arbiter.lock();
      std::printf("MuJoCo reset detected: OpenDog controller reset; motors locked\n");
    }
    previous_simulation_time = data->time;

    // 合并 GUI 失能锁存、窗口按键、终端按键和 GUI 按钮命令；按钮转换成等价按键。
    const bool pending_disable = simulation_running ?
      pending_disable_command.exchange(false) : false;
    const int pending_command = simulation_running ?
      pending_motion_command.exchange(kNoMotionCommand) : kNoMotionCommand;
    std::vector<teleop::OperatorCommand> commands;
    if (pending_disable) {commands.push_back({teleop::CommandType::DisableMotors});}
    const auto window_commands = keyboard.consumeWindow();
    commands.insert(commands.end(), window_commands.begin(), window_commands.end());
    const auto terminal_commands = keyboard.consume();
    commands.insert(commands.end(), terminal_commands.begin(), terminal_commands.end());
    if (pending_command == kForwardFast) {
      teleop::OperatorCommand command;
      command.type = teleop::CommandType::Motion;
      command.motion = teleop::Motion::ForwardFast;
      command.received_at = std::chrono::steady_clock::now();
      commands.push_back(command);
    } else {
      int key = 0;
      switch (pending_command) {
        case kEnableMotors: key = 'U'; break;
        case kDisableMotors: key = '0'; break;
        case kStand: key = '1'; break;
        case kMarchInPlace: key = '2'; break;
        case kTwoLegStand: key = '3'; break;
        case kForwardSlow: key = 'W'; break;
        case kBackward: key = 'S'; break;
        case kLeft: key = 'A'; break;
        case kRight: key = 'D'; break;
        case kRotateCounterClockwise: key = 'Q'; break;
        case kRotateClockwise: key = 'E'; break;
        case kProneDown: key = 'P'; break;
        case kStop: key = ' '; break;
        default: break;
      }
      if (key != 0) {commands.push_back(*teleop::decodeKey(key));}
    }
    arbiter.applyBatch(commands);
    for (const auto & command : commands) {
      if (command.type == teleop::CommandType::Motion && arbiter.motionActive() &&
        command.motion == arbiter.motion()) {++motion_sequence;}
    }
    if (keyboard.available() && !keyboard.healthy() && !keyboard_fault_reported) {
      keyboard_fault_reported = true;
      std::fprintf(stderr, "OpenDog MuJoCo keyboard input stopped; GUI control remains active\n");
    }
    arbiter.expireMotion(std::chrono::steady_clock::now());
    if (arbiter.takeHelpRequest()) {
      std::printf("OpenDog keys: U enable, 0 disable, 1 stand, 2 trot, 3 LF/RH single step, 4 RF/LH single step, W/S/A/D/Q/E locomotion, Space stop, Esc exit, H help\n");
    }
    if (arbiter.quitRequested()) {
      simulation.exitrequest.store(1);
      lock.unlock();
      break;
    }
    // 仿真中使能立即生效，并复位算法等待站立命令。
    if (arbiter.takeEnableRequest()) {
      arbiter.markEnabled();
      algorithm.reset();
      output = ControlOutput{};
      control_active = false;
      was_enabled = true;
      controller_ready = false;
      std::printf("OpenDog MuJoCo motors unlocked; press Stand up\n");
    }
    if (arbiter.state() != teleop::MotorOutputState::Enabled) {
      if (was_enabled) {algorithm.reset();}
      was_enabled = false;
      control_active = false;
      controller_ready = false;
      consecutive_failures = 0;
    }
    if (arbiter.takeStandUpRequest() &&
      arbiter.state() == teleop::MotorOutputState::Enabled)
    {
      desired.mode = ControlMode::StandUp;
      desired.body_position_world.z() = standing_height;
      desired.valid = true;
      control_active = true;
      std::printf("OpenDog stand-up request accepted\n");
    }
    // 停止：切到平衡站立，并锁定当前水平位置和航向。
    if (arbiter.takeStopRequest()) {
      desired.mode = ControlMode::BalanceStand;
      desired.body_velocity_world.setZero();
      desired.body_angular_velocity.setZero();
      if (output.estimate.valid) {
        desired.body_position_world.x() = output.estimate.position_world.x();
        desired.body_position_world.y() = output.estimate.position_world.y();
        desired.body_rpy.z() = output.estimate.rpy.z();
      }
      desired.body_position_world.z() = standing_height;
      desired.valid = true;
      control_active = true;
    }
    if (arbiter.takeProneDownRequest()) {
      std::printf("OpenDog MuJoCo target trajectory has no prone-down command\n");
    }
    // 运动：机身系速度档位按当前估计姿态旋转到世界系。
    if (arbiter.state() == teleop::MotorOutputState::Enabled && arbiter.motionActive()) {
      algorithm.setMotionRequest(arbiter.motion(), motion_sequence);
      const auto velocity = teleop::velocityForMotion(arbiter.motion());
      const Vec3<float> body_velocity(velocity.forward, velocity.lateral, 0.0F);
      desired.body_velocity_world =
        output.estimate.orientation_world_from_body * body_velocity;
      desired.body_angular_velocity.z() = velocity.yaw;
      desired.body_position_world.z() = standing_height;
      desired.mode = ControlMode::Locomotion;
      desired.valid = true;
      control_active = true;
    }

    if (simulation_running && control_active) {
      ControlInputSnapshot input;
      input.imu = sim_imu.read();
      for (std::size_t leg = 0; leg < kNumLegs; ++leg) {
        input.joints[leg] = sim_legs[leg]->read();
      }
      input.desired = desired;
      input.timestamp = static_cast<float>(data->time);
      // 控制器在快照之后不再访问 mjData。WBC/NMPC 计算期间释放渲染锁，
      // 否则计算耗时接近仿真步长时渲染线程拿不到锁，窗口会卡死。
      // 计算期间若发生复位/暂停，快照已失效，丢弃本帧输出，不把旧力矩施加到新状态。
      const double input_simulation_time = data->time;
      lock.unlock();
      const bool control_valid = algorithm.compute(input, output);
      lock.lock();
      const bool frame_still_current = !simulation.exitrequest.load() &&
        simulation.run != 0 &&
        std::abs(data->time - input_simulation_time) <= 0.5 * model->opt.timestep;
      if (frame_still_current) {
        if (control_valid) {
          consecutive_failures = 0;
          if (!controller_ready) {
            controller_ready = true;
            std::printf("OpenDog MuJoCo WBC+NMPC is producing valid commands\n");
          }
        } else if (++consecutive_failures % 500 == 0) {
          std::fprintf(stderr, "OpenDog rejected %zu consecutive simulation frames\n", consecutive_failures);
        }
        actuator_writer.write(output, data,
          arbiter.state() == teleop::MotorOutputState::Enabled && control_valid);
        mj_step(model, data);
      } else {
        actuator_writer.write(output, data, false);
        mj_forward(model, data);
      }
    } else if (simulation_running) {
      // 控制未激活：不施加力矩，仅推进物理。
      actuator_writer.write(output, data, false);
      mj_step(model, data);
    } else {
      // 暂停：只刷新派生量，不推进时间。
      actuator_writer.write(output, data, false);
      mj_forward(model, data);
    }
    lock.unlock();
    // 按物理步长限速；落后超过 4 个步长时放弃追赶，直接重新对齐。
    if (simulation_running) {
      const auto now = PhysicsClock::now();
      if (!was_running) {next_physics_deadline = now;}
      next_physics_deadline += physics_period;
      if (now < next_physics_deadline) {
        std::this_thread::sleep_until(next_physics_deadline);
      } else if (now - next_physics_deadline > physics_period * 4) {
        next_physics_deadline = now;
        std::this_thread::yield();
      } else {
        std::this_thread::yield();
      }
    } else {
      next_physics_deadline = PhysicsClock::now();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    was_running = simulation_running;
  }
  algorithm.stop();
}

/**
 * @brief 加载 MuJoCo 场景，并在物理线程中运行 OpenDog 控制循环。
 *
 * 渲染线程通过 mj::Simulate 管理窗口，物理线程负责读取 mjData、调用 OpenDog
 * 算法、写入关节力矩并推进 mj_step；共享的 mjModel/mjData 访问由
 * simulation.mtx 保护。
 */
void runPhysics(
  mj::Simulate & simulation, const std::string & scene_path,
  const std::string & algorithm_config_directory,
  std::atomic<int> & pending_motion_command, std::atomic_bool & pending_disable_command,
  teleop::KeyboardTeleop & keyboard, std::exception_ptr & failure)
{
  // 该函数运行在物理线程；渲染线程通过 simulation.mtx 与它共享 mjData。
  mjModel * model = nullptr;
  mjData * data = nullptr;
  try {
    // 读取 XML 并创建与模型匹配的运行时状态对象。
    char error[1024]{};
    model = mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error));
    if (model == nullptr) {
      throw std::runtime_error(std::string("unable to load MuJoCo scene: ") + error);
    }
    data = mj_makeData(model);
    if (data == nullptr) {throw std::runtime_error("unable to create MuJoCo data");}
    // home keyframe 同时提供初始 qpos、qvel 等状态，避免手工初始化遗漏。
    const int home_keyframe = mj_name2id(model, mjOBJ_KEY, "home");
    if (home_keyframe < 0) {
      throw std::runtime_error("MuJoCo model is missing the required home keyframe");
    }
    // DM1 的 home 是趴地零位，Reset 后仍从原地趴卧状态开始。
    mj_resetDataKeyframe(model, data, home_keyframe);
    mj_forward(model, data);
    // GUI 与无界面回归测试共用同一个执行器写入实现，避免双重位置控制。
    const SimulationActuatorWriter actuator_writer(model);

    // 将模型和数据交给 MuJoCo 的渲染器，使窗口能够显示当前物理状态。
    simulation.Load(model, data, scene_path.c_str());
    runOpenDogPhysics(
      simulation, model, data, actuator_writer, algorithm_config_directory,
      pending_motion_command, pending_disable_command, keyboard);
  } catch (...) {
    // 物理线程不能直接把异常抛到创建它的线程，因此保存 exception_ptr，
    // 让 run() 在 join 后重新抛出并交给上层统一处理。
    failure = std::current_exception();
    simulation.exitrequest.store(1);
  }

  // 无论正常退出还是异常退出，都必须释放 MuJoCo 的堆内存对象。
  if (data != nullptr) {mj_deleteData(data);}
  if (model != nullptr) {mj_deleteModel(model);}
}


/**
 * @brief 硬件在环仿真模式的物理循环。
 *
 * 这个循环只负责物理推进和输入转发，所有控制决策都由在独立线程中运行的
 * HardwareBridge 完成。GUI 暂停或复位时整个硬件桥会被停止并重新启动。
 */
void runHardwarePhysics(
  mj::Simulate & simulation, const std::string & scene_path,
  const HardwareSimulationOptions & settings, std::atomic<int> & pending_command,
  std::atomic_bool & pending_disable, teleop::KeyboardTeleop & keyboard,
  std::exception_ptr & failure)
{
  using Model = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
  using Data = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
  std::thread control;
  std::atomic_bool stop{false}, finished{false};
  std::exception_ptr control_failure;
  int result = 0;
  try {
    char error[1024]{};
    Model model(mj_loadXML(scene_path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
    if (!model) {throw std::runtime_error(error);}
    Data data(mj_makeData(model.get()), mj_deleteData);
    if (!data) {throw std::runtime_error("cannot allocate MuJoCo data");}
    const int home = mj_name2id(model.get(), mjOBJ_KEY, "home");
    if (home < 0) {throw std::runtime_error("DM1 home keyframe is missing");}
    mj_resetDataKeyframe(model.get(), data.get(), home);
    mj_forward(model.get(), data.get());
    const auto calibration = dm1_hardware::loadCalibration(settings.calibration_path);
    HardwareSimulation devices(model.get(), data.get(), simulation.mtx, calibration, settings);
    Dm1HardwareDriver driver("sim-can0", "sim-can1", devices.imuDevice(), calibration, &devices);
    HardwareBridge::Options options;
    options.keyboard_control = true;
    // 保持真机的时序、站高、安全阈值和命令默认值。
    options.torque_limit_nm = settings.torque_limit_nm;
    options.algorithm_config_directory = settings.algorithm_config_directory;
    // 把 GUI 按钮和键盘命令作为硬件桥的命令输入源。
    options.command_source = [&]() {
      auto commands = keyboard.consumeWindow();
      auto terminal = keyboard.consume();
      commands.insert(commands.end(), terminal.begin(), terminal.end());
      if (pending_disable.exchange(false)) {
        commands.push_back({teleop::CommandType::DisableMotors});
      }
      const int button = pending_command.exchange(kNoMotionCommand);
      int key = 0;
      switch (button) {
        case kEnableMotors: key = 'U'; break;
        case kDisableMotors: key = '0'; break;
        case kStand: key = '1'; break;
        case kMarchInPlace: key = '2'; break;
        case kTwoLegStand: key = '3'; break;
        case kProneDown: key = 'P'; break;
        case kStop: key = ' '; break;
        case kForwardSlow: key = 'W'; break;
        case kBackward: key = 'S'; break;
        case kLeft: key = 'A'; break;
        case kRight: key = 'D'; break;
        case kRotateCounterClockwise: key = 'Q'; break;
        case kRotateClockwise: key = 'E'; break;
        case kForwardFast: {
          teleop::OperatorCommand command;
          command.type = teleop::CommandType::Motion;
          command.motion = teleop::Motion::ForwardFast;
          command.received_at = std::chrono::steady_clock::now();
          commands.push_back(command);
          break;
        }
        default: break;
      }
      if (key) {commands.push_back(*teleop::decodeKey(key));}
      return commands;
    };
    // 在新线程中启动一个全新的硬件桥。
    auto start_control = [&]() {
      stop.store(false); finished.store(false); control_failure = nullptr;
      pending_command.store(kNoMotionCommand); pending_disable.store(false);
      keyboard.consumeWindow(); keyboard.consume();
      control = std::thread([&]() {
        try {
          HardwareBridge bridge(driver, calibration, options);
          result = bridge.run(stop);
        } catch (...) {control_failure = std::current_exception();}
        finished.store(true);
      });
    };
    // 停止硬件桥线程，并在设备侧立即失能全部电机。
    auto stop_control = [&]() {
      stop.store(true);
      if (control.joinable()) {control.join();}
      devices.emergencyStop();
    };
    simulation.Load(model.get(), data.get(), scene_path.c_str());
    std::printf(
      "DM1 HARDWARE SIMULATION: HardwareBridge + Dm1HardwareDriver + DmMotorDriver + "
      "HardwareImu; torque_limit=%.1f Nm, real control/sender=500 Hz.\n"
      "Unmeasured device assumptions: command/feedback/IMU delay=%.1f/%.1f/%.1f ms, "
      "IMU period=%.1f ms, motor response=%.1f ms. No physical CAN device is opened.\n",
      settings.torque_limit_nm, settings.command_delay_ms, settings.feedback_delay_ms,
      settings.imu_delay_ms, settings.imu_period_ms, settings.motor_time_constant_ms);
    start_control();
    double previous_time = data->time;
    bool paused = false;
    bool controller_fault = false;
    auto next = HardwareSimulation::Clock::now();
    const auto period = std::chrono::duration_cast<HardwareSimulation::Clock::duration>(
      std::chrono::duration<double>(model->opt.timestep));
    try {
      while (!simulation.exitrequest.load()) {
        bool running, reset;
        {
          std::lock_guard<std::recursive_mutex> lock(simulation.mtx);
          running = simulation.run != 0;
          reset = data->time + model->opt.timestep * 0.5 < previous_time;
          if (reset || (!running && !paused)) {devices.emergencyStop(); stop.store(true);}
        }
        if (reset || (!running && !paused)) {
          stop_control();
          if (reset) {
            std::lock_guard<std::recursive_mutex> lock(simulation.mtx);
            mj_resetDataKeyframe(model.get(), data.get(), home);
            mj_forward(model.get(), data.get());
            controller_fault = false;
            std::printf("Hardware simulation reset: entire hardware bridge restarted; motors locked.\n");
          }
          paused = true;
        }
        if (running && paused) {start_control(); paused = false;}
        // 硬件桥退出：正常退出则关闭仿真，异常退出码则停在故障状态等待复位。
        if (!paused && finished.load() && !controller_fault) {
          if (control_failure) {std::rethrow_exception(control_failure);}
          if (result == 0) {simulation.exitrequest.store(1);}
          else {
            controller_fault = true;
            devices.emergencyStop();
            std::fprintf(stderr, "HardwareBridge stopped with code %d; inspect scene/logs, then Reset.\n", result);
          }
        }
        {
          std::lock_guard<std::recursive_mutex> lock(simulation.mtx);
          devices.tick(running);
          if (running) {mj_step(model.get(), data.get());}
          else {mj_forward(model.get(), data.get());}
          previous_time = data->time;
        }
        next += period;
        const auto now = HardwareSimulation::Clock::now();
        if (next < now) {next = now;}
        std::this_thread::sleep_until(next);
      }
    } catch (...) {stop_control(); throw;}
    stop_control();
  } catch (...) {
    failure = std::current_exception();
    simulation.exitrequest.store(1);
  }
}

}  // namespace

SimulationBridge::SimulationBridge(std::string scene_path)
: scene_path_(std::move(scene_path))
{}

// 同时更新硬件在环模式使用的配置目录。
void SimulationBridge::setAlgorithmConfigDirectory(std::string config_directory)
{
  algorithm_config_directory_ = std::move(config_directory);
  hardware_simulation_.algorithm_config_directory = algorithm_config_directory_;
}

void SimulationBridge::setHardwareSimulation(const HardwareSimulationOptions & options)
{
  hardware_simulation_ = options;
}

/**
 * @brief 创建 MuJoCo Simulate 对象，启动物理线程并进入渲染循环。
 *
 * 主线程执行 RenderLoop，物理线程执行 runPhysics。RenderLoop 返回后等待
 * 物理线程结束，再清理全局回调指针；如果物理线程捕获到异常，此处重新抛出。
 */
int SimulationBridge::run()
{
  // 在创建线程和窗口前尽早检查配置错误。
  if (mjVERSION_HEADER != mj_version()) {
    throw std::runtime_error("MuJoCo headers and runtime library versions differ");
  }

  // 初始化 MuJoCo 相机、显示选项和交互扰动对象。
  mjvCamera camera;
  mjv_defaultCamera(&camera);
  mjvOption options;
  mjv_defaultOption(&options);
  mjvPerturb perturbation;
  mjv_defaultPerturb(&perturbation);
  // GlfwAdapter 负责创建 GLFW/OpenGL 窗口，Simulate 负责后续渲染和 UI。
  teleop::KeyboardTeleop keyboard;
  keyboard.start();
  auto glfw_adapter = std::make_unique<TeleopGlfwAdapter>(&keyboard, hardware_simulation_.enabled);
  const auto * gl_vendor = reinterpret_cast<const char *>(glGetString(GL_VENDOR));
  const auto * gl_renderer = reinterpret_cast<const char *>(glGetString(GL_RENDERER));
  std::printf(
    "OpenGL renderer: vendor=%s, device=%s\n",
    gl_vendor == nullptr ? "unknown" : gl_vendor,
    gl_renderer == nullptr ? "unknown" : gl_renderer);
  const char * requested_gl_vendor = std::getenv("__GLX_VENDOR_LIBRARY_NAME");
  if (requested_gl_vendor != nullptr &&
    std::strcmp(requested_gl_vendor, "nvidia") == 0 &&
    (gl_vendor == nullptr || std::strstr(gl_vendor, "NVIDIA") == nullptr))
  {
    std::fprintf(
      stderr,
      "NVIDIA rendering was requested but GLX selected %s. "
      "Install the matching NVIDIA OpenGL userspace package "
      "(libnvidia-gl-595 on this machine), then log out and back in.\n",
      gl_vendor == nullptr ? "an unknown renderer" : gl_vendor);
  }
  // UI 回调没有 this 指针，因此在 RenderLoop 期间把成员地址注册到文件内全局指针。
  g_hardware_simulation = hardware_simulation_.enabled;
  g_pending_motion_command = &pending_motion_command_;
  g_pending_disable_command = &pending_disable_command_;
  auto simulation = std::make_unique<mj::Simulate>(
    std::move(glfw_adapter), &camera, &options, &perturbation, false);
  simulation->run = true;

  std::exception_ptr failure;
  // 请求和 UI 状态按引用共享，并通过 atomic 或互斥保护。
  std::thread physics;
  if (hardware_simulation_.enabled) {
    physics = std::thread(runHardwarePhysics, std::ref(*simulation), std::cref(scene_path_),
      std::cref(hardware_simulation_), std::ref(pending_motion_command_),
      std::ref(pending_disable_command_), std::ref(keyboard), std::ref(failure));
  } else {
    physics = std::thread(
      runPhysics, std::ref(*simulation), std::cref(scene_path_),
      std::cref(algorithm_config_directory_),
      std::ref(pending_motion_command_), std::ref(pending_disable_command_),
      std::ref(keyboard), std::ref(failure));
  }
  // 当前线程进入渲染循环，直到用户关闭窗口或物理线程发生异常。
  simulation->RenderLoop();
  physics.join();
  // RenderLoop 结束后不再允许 UI 回调访问这些成员地址。
  g_pending_motion_command = nullptr;
  g_pending_disable_command = nullptr;
  if (failure != nullptr) {std::rethrow_exception(failure);}
  return 0;
}
