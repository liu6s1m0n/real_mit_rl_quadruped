/**
 * @file SimulationBridge.cpp
 * @brief MuJoCo 仿真适配层的实现。
 *
 * 本文件连接 MuJoCo 的渲染/UI 线程、物理线程和机器人控制器：
 * UI 线程产生用户命令，物理线程读取机器人状态并调用 RobotRunner，
 * 最后将关节力矩写回 MuJoCo 并推进仿真时间。
 */
#include "SimulationBridge.hpp"
#include "SimulationActuatorWriter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <glfw_adapter.h>
#include <mujoco/mujoco.h>
#include <simulate.h>

#include <dlfcn.h>
#include <sys/socket.h>
#include <unistd.h>

#include "RobotRunner.hpp"
#include "SimulationDiagnostics.hpp"
#include "StandingHeightIpc.hpp"
#include "controller/frozen_dwaq_policy.hpp"
#include "model/robot_control_parameters.hpp"

namespace mj = ::mujoco;

namespace
{

/** MuJoCo UI 添加函数的原始函数指针类型。 */
using MjuiAddFunction = void (*)(mjUI *, const mjuiDef *);
/** MuJoCo UI 事件处理函数的原始函数指针类型。 */
using MjuiEventFunction = mjuiItem * (*)(mjUI *, mjuiState *, const mjrContext *);

// 这些指针在 SimulationBridge::run() 启动 RenderLoop 期间有效。
// C 风格的 MuJoCo 回调通过它们访问主对象中的共享状态。
double * g_standing_height_slider = nullptr;
std::atomic<int> * g_pending_motion_command = nullptr;

enum MotionCommand : int
{
  kNoMotionCommand = -1,
  kStand = 0,
  kForwardSlow = 1,
  kForwardFast = 2,
  kBackward = 3,
  kLeft = 4,
  kRight = 5,
  kRotate = 6,
  kSelectMpc = 7,
  kSelectRl = 8
};

int commandForButton(const char * name)
{
  if (name == nullptr) {return kNoMotionCommand;}
  constexpr std::array<const char *, 9> names{
    "Stand up", "Forward slow", "Forward fast", "Backward",
    "Left", "Right", "Rotate CCW", "Use MPC", "Use RL"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (std::strcmp(name, names[index]) == 0) {
      return static_cast<int>(index);
    }
  }
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
  if (g_standing_height_slider == nullptr || g_pending_motion_command == nullptr ||
    definition == nullptr ||
    definition[0].type != mjITEM_SECTION ||
    std::strcmp(definition[0].name, "Simulation") != 0)
  {
    return;
  }

  // mjuiDef 数组必须以 mjITEM_END 结束；控件数据直接绑定到共享状态。
  const mjuiDef height_controls[] = {
    {mjITEM_SEPARATOR, "Robot controller", 1, nullptr, "", 0},
    {
      mjITEM_SLIDERNUM, "Height (m)", 2,
      g_standing_height_slider, "0.30 0.42", 0
    },
    {mjITEM_BUTTON, "Forward slow", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Forward fast", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Backward", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Left", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Right", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Rotate CCW", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Use MPC", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Use RL", 2, nullptr, "", 0},
    {mjITEM_BUTTON, "Stand up", 2, nullptr, "", 0},
    {mjITEM_END, "", 0, nullptr, "", 0}
  };
  add(ui, height_controls);
}

/**
 * @brief 包装 MuJoCo UI 事件，并转发一次性动作请求。
 *
 * MuJoCo 原函数负责处理鼠标点击和控件状态。本函数只检查返回的控件，
 * 通过 atomic 命令通知物理线程；物理线程用 exchange(kNoMotionCommand)
 * 消费一次性请求。
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
    g_pending_motion_command != nullptr)
  {
    const int command = commandForButton(changed->name);
    if (command != kNoMotionCommand) {
      g_pending_motion_command->store(command);
    }
  }
  return changed;
}

namespace
{

class StandingHeightReceiver
{
public:
  /**
   * @brief 创建非阻塞 Unix 域数据报接收端。
   *
   * 外部 ROS 接口可以向该 socket 发送目标站立高度；物理线程调用 receive()
   * 时不会因没有新消息而阻塞控制循环。
   */
  StandingHeightReceiver()
  : descriptor_(::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0))
  {
    if (descriptor_ < 0) {
      throw std::runtime_error(
              std::string("unable to create height-command receiver: ") +
              std::strerror(errno));
    }
    const sockaddr_un address = standing_height_ipc::socketAddress();
    if (::bind(
        descriptor_, reinterpret_cast<const sockaddr *>(&address),
        standing_height_ipc::socketAddressLength()) < 0)
    {
      const std::string message = std::string("unable to bind height-command receiver: ") +
        std::strerror(errno);
      ::close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error(message);
    }
  }

  /** @brief 关闭 socket，释放本地 IPC 资源。 */
  ~StandingHeightReceiver()
  {
    if (descriptor_ >= 0) {::close(descriptor_);}
  }

  StandingHeightReceiver(const StandingHeightReceiver &) = delete;
  StandingHeightReceiver & operator=(const StandingHeightReceiver &) = delete;

  /**
   * @brief 非阻塞读取所有待处理高度消息，并保留最后一条有效命令。
   * @param standing_height 写入控制器使用的原子高度目标。
   */
  void receive(std::atomic<float> & standing_height) const
  {
    // 非阻塞地读完队列，只保留最后到达的有效高度；没有数据时立即返回物理循环。
    standing_height_ipc::Command command;
    while (true) {
      const ssize_t received = ::recv(descriptor_, &command, sizeof(command), 0);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {return;}
        if (errno == EINTR) {continue;}
        throw std::runtime_error(
                std::string("unable to receive height command: ") +
                std::strerror(errno));
      }
      if (received != static_cast<ssize_t>(sizeof(command)) ||
        command.magic != standing_height_ipc::kCommandMagic ||
        !std::isfinite(command.height) ||
        command.height < standing_height_ipc::kMinimumHeight ||
        command.height > standing_height_ipc::kMaximumHeight)
      {
        std::fprintf(stderr, "ignored an invalid standing-height command\n");
        continue;
      }
      standing_height.store(command.height);
      std::printf("Standing-height target set to %.3f m\n", command.height);
    }
  }

private:
  int descriptor_ = -1;
};

constexpr std::array<LegId, kNumLegs> kLegOrder{
  LegId::FR, LegId::FL, LegId::RR, LegId::RL};

/**
 * @brief 合并 UI 滑块和外部 IPC 的站立高度命令。
 *
 * 如果滑块相对上一帧发生变化，认为用户正在操作 UI，滑块值优先；
 * 否则检查 ROS/IPC 是否写入了新的原子目标，并把它同步回滑块。
 * @return 当前最终采用的高度目标。
 */
float synchronizeStandingHeight(
  double & slider_height, float & previous_slider_height,
  std::atomic<float> & requested_height)
{
  // 滑块变化时以 GUI 为准；否则把 ROS 服务写入的原子目标同步回滑块。
  const float slider_value = std::clamp(
    static_cast<float>(slider_height),
    standing_height_ipc::kMinimumHeight,
    standing_height_ipc::kMaximumHeight);
  if (std::abs(slider_value - previous_slider_height) > 1.0e-6F) {
    requested_height.store(slider_value);
    slider_height = slider_value;
  } else {
    const float external_value = requested_height.load();
    if (std::abs(external_value - previous_slider_height) > 1.0e-6F) {
      slider_height = external_value;
    }
  }

  previous_slider_height = static_cast<float>(slider_height);
  return previous_slider_height;
}

/**
 * @brief 运行 MuJoCo 物理、状态估计和机器人控制循环。
 *
 * 本函数在线程中执行。渲染线程通过 mj::Simulate 管理窗口，物理线程负责
 * 读取 mjData、调用 RobotRunner、写入关节力矩并推进 mj_step；共享的
 * mjModel/mjData 访问由 simulation.mtx 保护。
 *
 * 每个控制周期的主要顺序为：加载模型 -> 恢复 home -> 读取 UI/IPC 命令
 * -> 运行控制器 -> 写入力矩 -> 推进仿真；暂停时只调用 mj_forward 刷新显示。
 */
void runPhysics(
  mj::Simulate & simulation, const std::string & scene_path, RobotType robot_type,
  std::atomic<float> & standing_height, double & standing_height_slider,
  std::atomic<int> & pending_motion_command, float slow_walking_forward_speed,
  float fast_walking_forward_speed, float walking_backward_speed,
  float walking_lateral_speed, float turning_yaw_rate,
  ControlMode initial_walking_mode, const RlPolicyPtr & rl_policy,
  std::exception_ptr & failure)
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
    // RobotRunner 负责状态估计、步态/WBC 等控制流程；本文件只负责仿真桥接。
    RobotRunner runner(model, data, robot_type);
    // 只有 RL 模式会使用这个策略；MPC 模式仍走原有 Locomotion 路径。
    runner.setRlPolicy(rl_policy);
    SimulationDiagnostics diagnostics(model);
    StandingHeightReceiver height_receiver;

    // 将模型和数据交给 MuJoCo 的渲染器，使窗口能够显示当前物理状态。
    simulation.Load(model, data, scene_path.c_str());
    std::printf(
      "MuJoCo %s control started: dt=%.4f s, slow/fast=%.2f/%.2f m/s, "
      "backward=%.2f m/s, lateral=%.2f m/s, turning=%.2f rad/s, "
      "default mode=%s\n",
      "DM1",
      model->opt.timestep, slow_walking_forward_speed, fast_walking_forward_speed,
      walking_backward_speed, walking_lateral_speed, turning_yaw_rate,
      // 换模型要修改的地方：启动日志模型编号
      initial_walking_mode == ControlMode::WalkRl ? "RL model_4210" : "MPC");

    bool controller_ready = false;
    std::size_t consecutive_failures = 0;
    double previous_simulation_time = data->time;
    float previous_slider_height = standing_height.load();
    int previous_direction = -1;
    ControlMode walking_mode = initial_walking_mode;
    Vec3<float> command_body = Vec3<float>::Zero();
    // 物理仿真按模型 timestep 与墙钟同步。Release 构建通常会提前完成控制计算，
    // 剩余时间专门留给 GLFW 渲染线程，避免物理线程无限抢占共享锁。
    using PhysicsClock = std::chrono::steady_clock;
    const auto physics_period = std::chrono::duration_cast<PhysicsClock::duration>(
      std::chrono::duration<double>(model->opt.timestep));
    auto next_physics_deadline = PhysicsClock::now();
    bool was_running = false;
    while (!simulation.exitrequest.load()) {
      // RenderLoop 也会访问 model/data；锁覆盖当前帧的读取、写入和 mj_step。
      std::unique_lock<std::recursive_mutex> lock(simulation.mtx);
      const bool simulation_running = simulation.run != 0;
      // MuJoCo Reset 会让仿真时间回退，据此重置估计器和 FSM 内部历史。
      if (data->time + 0.5 * model->opt.timestep < previous_simulation_time) {
        // Reset 后 DM1 回到趴地 home，不从空中重新生成机器人。
        mj_resetDataKeyframe(model, data, home_keyframe);
        mj_forward(model, data);
        runner.reset();
        diagnostics.reset();
        controller_ready = false;
        consecutive_failures = 0;
        previous_direction = -1;
        std::printf(
          "MuJoCo reset detected: robot and controller restored to home posture\n");
      }
      previous_simulation_time = data->time;
      // 高度命令有两条来源：MuJoCo 滑块和 ROS/Unix socket。
      height_receiver.receive(standing_height);
      float commanded_height = synchronizeStandingHeight(
        standing_height_slider, previous_slider_height, standing_height);
      commanded_height = std::clamp(
        commanded_height, runner.minimumStandingHeight(), runner.maximumStandingHeight());
      standing_height.store(commanded_height);
      standing_height_slider = commanded_height;
      // 方向控件是一次性按钮：点击后锁存该速度档，直到再次点击另一个
      // 方向或 Stand up。这样不会因为复选框状态同步产生“幽灵”运动命令。
      int requested_direction = previous_direction;
      const int pending_command = simulation_running ?
        pending_motion_command.exchange(kNoMotionCommand) : kNoMotionCommand;
      if (pending_command == kSelectMpc || pending_command == kSelectRl) {
        if (previous_direction < 0) {
          walking_mode = pending_command == kSelectRl ?
            ControlMode::WalkRl : ControlMode::Locomotion;
          std::printf(
            "Walking controller selected: %s (takes effect on the next direction command)\n",
            // 换模型要修改的地方：切换日志模型编号
            walking_mode == ControlMode::WalkRl ? "RL model_4210" : "MPC");
        } else {
          std::printf(
            "Controller selection ignored while walking; press Stand up first\n");
        }
      }
      if (pending_command == kStand) {
        requested_direction = -1;
      } else if (pending_command > kStand && pending_command < kSelectMpc) {
        // MotionCommand 的编号与下面的 switch 共用同一组枚举值；不能再
        // 减一，否则按钮会整体错位（Left 变成 Backward 等）。
        requested_direction = pending_command;
      }

      // 只有方向发生变化时才向 RobotRunner 重发命令，避免每帧重复切换状态机。
      if (requested_direction != previous_direction) {
        if (requested_direction >= 0 && !runner.standingReady()) {
          requested_direction = -1;
          std::printf("Motion ignored: press Stand up and wait for BalanceStand\n");
        }
        command_body.setZero();
        const char * direction_name = "Stand";
        switch (requested_direction) {
          case kForwardSlow:
            command_body.x() = slow_walking_forward_speed;
            direction_name = "Forward slow";
            break;
          case kForwardFast:
            command_body.x() = fast_walking_forward_speed;
            direction_name = "Forward fast";
            break;
          case kBackward:
            command_body.x() = -walking_backward_speed;
            direction_name = "Backward";
            break;
          case kLeft:
            command_body.y() = walking_lateral_speed;
            direction_name = "Left";
            break;
          case kRight:
            command_body.y() = -walking_lateral_speed;
            direction_name = "Right";
            break;
          case kRotate:
            command_body.z() = turning_yaw_rate;
            direction_name = "Rotate CCW";
            break;
          default:
            break;
        }
        // 将 UI 方向转换成控制器使用的机身速度命令。
        runner.setLocomotionVelocityCommand(
          command_body.x(), command_body.y(), command_body.z());
        runner.setControlMode(
          requested_direction < 0 ?
          ControlMode::BalanceStand : walking_mode);
        // 每次切换方向重新开始一段统计，方便直接观察该命令下的估计误差、
        // 俯仰和小腿碰地情况，而不是被上一方向的累计峰值污染。
        diagnostics.reset();
        std::printf("Robot direction set to %s\n", direction_name);
        previous_direction = requested_direction;
      }
      if (simulation_running) {
        // exchange(false) 具有“读取并清零”的原子语义，避免按钮请求重复执行。
        if (pending_command == kStand) {
          if (runner.requestStandUp()) {
            previous_direction = -1;
            diagnostics.reset();
            std::printf("Stand-up request accepted\n");
          } else {
            std::printf("Stand up ignored: initialization incomplete or robot already standing\n");
          }
        }
        // 控制计算使用当前传感器状态，随后写力矩，最后推进一个物理时间步。
        // BalanceStand/MPC follows the external height command. RL walking has
        // one owner for its training posture: RobotRunner starts the 0.39 ->
        // 0.38 m transition and keeps that target after entry, so the GUI
        // height slider cannot overwrite the RL contract every frame.
        if (!(walking_mode == ControlMode::WalkRl && requested_direction > kStand)) {
          runner.setStandingHeight(commanded_height);
        }
        const bool control_valid = runner.run();
        if (control_valid) {
          consecutive_failures = 0;
          if (!controller_ready) {
            controller_ready = true;
            std::printf("RobotRunner control pipeline is producing valid commands\n");
          }
        } else if (++consecutive_failures % 500 == 0) {
          std::fprintf(
            stderr, "RobotRunner has rejected %zu consecutive control frames\n",
            consecutive_failures);
        }
        const std::size_t target_joint_limit_hits =
          countSimulationTargetJointLimitHits(runner);
        const bool direction_active =
          requested_direction > kStand && requested_direction < kSelectMpc &&
          runner.currentStateName() == FSM_StateName::LOCOMOTION;
        const std::array<float, kRlActionSize> * raw_action =
          walking_mode == ControlMode::WalkRl && direction_active &&
          runner.hasRlRawAction() ? &runner.rlLastRawAction() : nullptr;
        std::array<std::size_t, kNumLegs> torque_speed_saturation_by_leg{};
        actuator_writer.write(runner, data, torque_speed_saturation_by_leg);
        diagnostics.observe(
          data, runner.stateEstimate(), control_valid, direction_active, command_body,
          raw_action,
          target_joint_limit_hits, &torque_speed_saturation_by_leg);
        const auto & diagnostic_report = diagnostics.report();
        if (diagnostic_report.observed_frames != 0 &&
          diagnostic_report.observed_frames % 500 == 0)
        {
          const auto mean_1_2 = diagnostic_report.direction_windows[1].meanActual();
          const auto mean_2_5 = diagnostic_report.direction_windows[2].meanActual();
          const auto mean_5_10 = diagnostic_report.direction_windows[3].meanActual();
          // 500 Hz下每秒输出一次正式运行诊断。这里只报告，不改变控制状态。
          std::printf(
            "Runtime diagnostics: pos_rms=%.4f m, vel_rms=%.4f m/s, "
            "pitch_max=%.3f rad, height_min=%.3f m, calf_contacts=%zu, "
            "rejected=%zu, cmd=(%.3f,%.3f,%.3f), actual_body=(%.3f,%.3f,%.3f), "
            "mean_body_1_2=(%.3f,%.3f,%.3f), mean_body_2_5=(%.3f,%.3f,%.3f), "
            "mean_body_5_10=(%.3f,%.3f,%.3f), "
            "raw_max=%.3f, raw_over1_ratio=%.3f, joint_limits=%zu, "
            "torque_speed_sat=%zu, foot_slip_mean=%.3f, fall_time=%.3f, "
            "net_body=(%.3f,%.3f), contact_transitions=(%zu,%zu,%zu,%zu)\n",
            diagnostic_report.rmsPositionError(),
            diagnostic_report.rmsVelocityError(),
            diagnostic_report.maximum_absolute_pitch,
            diagnostic_report.minimum_height,
            diagnostic_report.calf_collision_frames,
            diagnostic_report.rejected_control_frames,
            diagnostic_report.command_vx, diagnostic_report.command_vy,
            diagnostic_report.command_wz, diagnostic_report.actual_vx,
            diagnostic_report.actual_vy, diagnostic_report.actual_wz,
            mean_1_2[0], mean_1_2[1], mean_1_2[2],
            mean_2_5[0], mean_2_5[1], mean_2_5[2],
            mean_5_10[0], mean_5_10[1], mean_5_10[2],
            diagnostic_report.raw_action_max_abs,
            diagnostic_report.rawActionOverOneRatio(),
            diagnostic_report.target_joint_limit_hits,
            diagnostic_report.torque_speed_saturation_count,
            diagnostic_report.meanFootSlipSpeed(), diagnostic_report.fall_time_s,
            diagnostic_report.net_displacement_body_x,
            diagnostic_report.net_displacement_body_y,
            diagnostic_report.foot_contact_by_leg[0].contact_transitions,
            diagnostic_report.foot_contact_by_leg[1].contact_transitions,
            diagnostic_report.foot_contact_by_leg[2].contact_transitions,
            diagnostic_report.foot_contact_by_leg[3].contact_transitions);
        }
        // 当前命令已写入启用的关节，随后推进一个 MuJoCo 动力学时间步。
        mj_step(model, data);
      } else {
        // 暂停时只刷新运动学量，不推进时间，也不运行控制器。
        mj_forward(model, data);
      }
      // 必须先释放共享锁再等待，否则 RenderLoop 会被物理线程一起阻塞。
      lock.unlock();
      if (simulation_running) {
        const auto now = PhysicsClock::now();
        if (!was_running) {next_physics_deadline = now;}
        next_physics_deadline += physics_period;
        if (now < next_physics_deadline) {
          std::this_thread::sleep_until(next_physics_deadline);
        } else if (now - next_physics_deadline > physics_period * 4) {
          // 调试断点、窗口拖动等长暂停后从当前墙钟重新同步，不连续追赶旧帧。
          next_physics_deadline = now;
          std::this_thread::yield();
        } else {
          // 单帧轻微超时时仍让渲染线程获得调度机会。
          std::this_thread::yield();
        }
      } else {
        next_physics_deadline = PhysicsClock::now();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      was_running = simulation_running;
    }
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

}  // namespace

/**
 * @brief 保存场景路径和机器人型号，并检查路径参数。
 */
SimulationBridge::SimulationBridge(std::string scene_path, RobotType robot_type)
: scene_path_(std::move(scene_path)), robot_type_(robot_type)
{
  if (scene_path_.empty()) {throw std::invalid_argument("scene path must not be empty");}
  // 预先准备 RL 后端，便于程序启动为 MPC 后仍可通过界面切到 RL。
  // MPC 路径不会调用它，也不会改变原有 MPC/WBC 计算。
  rl_policy_ = std::make_shared<FrozenDwaqPolicy>();
}

/**
 * @brief 设置站立高度，并同步更新 GUI 滑块的初始值。
 */
void SimulationBridge::setStandingHeight(float height)
{
  const auto parameters = makeRobotControlParameters<float>(robot_type_);
  if (!std::isfinite(height) || height < parameters.minimum_standing_height ||
    height > parameters.maximum_standing_height)
  {
    throw std::invalid_argument("standing height is outside the selected robot profile");
  }
  standing_height_.store(height);
  // run() 启动物理线程前此处仅由 main 调用，因此可同步初始化 UI 缓存。
  standing_height_slider_ = height;
}

/** @brief 设置低速前进档，并限制在允许的安全范围内。 */
void SimulationBridge::setSlowWalkingForwardSpeed(float speed)
{
  if (!std::isfinite(speed) || speed <= 0.0F || speed > 0.6F) {
    throw std::invalid_argument("slow walking speed must be within (0, 0.6] m/s");
  }
  slow_walking_forward_speed_ = speed;
}

/** @brief 设置快速前进档，并限制在允许的安全范围内。 */
void SimulationBridge::setFastWalkingForwardSpeed(float speed)
{
  if (!std::isfinite(speed) || speed <= 0.0F || speed > 0.6F) {
    throw std::invalid_argument("fast walking speed must be within (0, 0.6] m/s");
  }
  fast_walking_forward_speed_ = speed;
}

/** @brief 设置横向移动档，并限制横向速度的绝对值。 */
void SimulationBridge::setWalkingLateralSpeed(float speed)
{
  if (!std::isfinite(speed) || speed < 0.0F || speed > 0.6F) {
    throw std::invalid_argument("lateral speed must be within [0, 0.6] m/s");
  }
  walking_lateral_speed_ = speed;
}

/** @brief 设置原地旋转档，并限制偏航角速度的绝对值。 */
void SimulationBridge::setTurningYawRate(float yaw_rate)
{
  if (!std::isfinite(yaw_rate) || yaw_rate < 0.0F || yaw_rate > 1.5F) {
    throw std::invalid_argument("turning yaw rate must be within [0, 1.5] rad/s");
  }
  turning_yaw_rate_ = yaw_rate;
}

void SimulationBridge::setWalkingControllerMode(ControlMode mode)
{
  if (mode == ControlMode::Locomotion) {
    walking_mode_ = mode;
    return;
  }
  if (mode == ControlMode::WalkRl) {
    walking_mode_ = mode;
    rl_policy_ = std::make_shared<FrozenDwaqPolicy>();
    return;
  }
  throw std::invalid_argument(
          "walking controller mode must be ControlMode::Locomotion (MPC) or WalkRl (RL)");
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
  if (slow_walking_forward_speed_ >= fast_walking_forward_speed_) {
    throw std::runtime_error("slow walking speed must be lower than fast walking speed");
  }

  // 初始化 MuJoCo 相机、显示选项和交互扰动对象。
  mjvCamera camera;
  mjv_defaultCamera(&camera);
  mjvOption options;
  mjv_defaultOption(&options);
  mjvPerturb perturbation;
  mjv_defaultPerturb(&perturbation);
  // GlfwAdapter 负责创建 GLFW/OpenGL 窗口，Simulate 负责后续渲染和 UI。
  auto glfw_adapter = std::make_unique<mj::GlfwAdapter>();
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
  g_standing_height_slider = &standing_height_slider_;
  g_pending_motion_command = &pending_motion_command_;
  auto simulation = std::make_unique<mj::Simulate>(
    std::move(glfw_adapter), &camera, &options, &perturbation, false);
  simulation->run = true;

  std::exception_ptr failure;
  // 速度参数按值传给物理线程；请求和 UI 状态按引用共享，并通过 atomic 或互斥保护。
  std::thread physics(
    runPhysics, std::ref(*simulation), std::cref(scene_path_), robot_type_,
    std::ref(standing_height_), std::ref(standing_height_slider_),
    std::ref(pending_motion_command_), slow_walking_forward_speed_,
    fast_walking_forward_speed_, walking_backward_speed_, walking_lateral_speed_,
    turning_yaw_rate_, walking_mode_, rl_policy_, std::ref(failure));
  // 当前线程进入渲染循环，直到用户关闭窗口或物理线程发生异常。
  simulation->RenderLoop();
  physics.join();
  // RenderLoop 结束后不再允许 UI 回调访问这些成员地址。
  g_standing_height_slider = nullptr;
  g_pending_motion_command = nullptr;
  if (failure != nullptr) {std::rethrow_exception(failure);}
  return 0;
}
