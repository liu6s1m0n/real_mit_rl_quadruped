// 正式仿真入口：这里配置运行参数，SimulationBridge 再将其送入
// RobotRunner -> ControlFSM -> MPC/WBC -> LegController 控制链。
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

#include "SimulationBridge.hpp"

#if defined(MYMIT_ENABLE_OPENDOG)
#include <rclcpp/rclcpp.hpp>
#endif

namespace
{
// 软件控制默认值统一由 config/control_config.h 提供。

struct RobotSelection
{
  const char * scene_path;    ///< DM1 MuJoCo 场景。
  const char * render_gpu;    ///< OpenGL 渲染设备策略：auto 或 nvidia。
  HardwareSimulationOptions hardware_simulation;
  AlgorithmBackendId algorithm_backend = AlgorithmBackendId::CurrentMit;
  std::string algorithm_config_directory = "config/algorithm";
};

RobotSelection selectRobot(int argc, char ** argv)
{
  // launch_ros 会在自定义参数后追加“--ros-args -r ...”。这里只解析渲染
  // 选项，遇到 --ros-args 后把剩余参数完整留给 ROS 2。
  const char * render_gpu = "auto";
  HardwareSimulationOptions hardware_simulation;
  hardware_simulation.calibration_path = MYMIT_ROBOT_DM1_CALIBRATION_PATH;
  AlgorithmBackendId algorithm_backend = AlgorithmBackendId::CurrentMit;
#ifdef MYMIT_ROBOT_ALGORITHM_CONFIG_DIR
  std::string algorithm_config_directory = MYMIT_ROBOT_ALGORITHM_CONFIG_DIR;
#else
  std::string algorithm_config_directory = "config/algorithm";
#endif
  bool hardware_options_used = false;
  for (int index = 1; index < argc; ++index) {
    if (std::strcmp(argv[index], "--ros-args") == 0) {break;}
    if (std::strcmp(argv[index], "--algorithm") == 0) {
      if (++index >= argc) {throw std::invalid_argument("--algorithm requires current_mit or opendog");}
      const std::string value(argv[index]);
      if (value == "current_mit" || value == "mit") {
        algorithm_backend = AlgorithmBackendId::CurrentMit;
      } else if (value == "opendog") {
        algorithm_backend = AlgorithmBackendId::OpenDog;
      } else {
        throw std::invalid_argument("--algorithm must be current_mit or opendog");
      }
    } else if (std::strcmp(argv[index], "--algorithm-config") == 0) {
      if (++index >= argc) {throw std::invalid_argument("--algorithm-config requires a path");}
      algorithm_config_directory = argv[index];
    } else if (std::strcmp(argv[index], "--hardware-sim") == 0) {
      hardware_simulation.enabled = true;
    } else if (std::strcmp(argv[index], "--calibration") == 0) {
      hardware_options_used = true;
      if (++index >= argc) {throw std::invalid_argument("--calibration requires a path");}
      hardware_simulation.calibration_path = argv[index];
    } else if (std::strncmp(argv[index], "--sim-", 6) == 0 ||
      std::strcmp(argv[index], "--torque-limit") == 0)
    {
      const std::string option(argv[index]);
      hardware_options_used = true;
      if (++index >= argc) {throw std::invalid_argument(option + " requires a number");}
      std::size_t parsed = 0;
      const double value = std::stod(argv[index], &parsed);
      if (argv[index][parsed] != '\0') {throw std::invalid_argument("invalid number for " + option);}
      if (option == "--sim-command-delay-ms") {hardware_simulation.command_delay_ms = value;}
      else if (option == "--sim-feedback-delay-ms") {hardware_simulation.feedback_delay_ms = value;}
      else if (option == "--sim-imu-period-ms") {hardware_simulation.imu_period_ms = value;}
      else if (option == "--sim-imu-delay-ms") {hardware_simulation.imu_delay_ms = value;}
      else if (option == "--sim-motor-time-constant-ms") {hardware_simulation.motor_time_constant_ms = value;}
      else if (option == "--sim-output-scale") {hardware_simulation.output_scale = value;}
      else if (option == "--sim-impairment-after-s") {hardware_simulation.impairment_after_s = value;}
      else if (option == "--sim-drop-every") {hardware_simulation.drop_every = value;}
      else if (option == "--torque-limit") {hardware_simulation.torque_limit_nm = value;}
      else if (option == "--sim-weak-joint") {
        if (!std::isfinite(value) || value < 0 || value >= kNumJoints || std::floor(value) != value) {
          throw std::invalid_argument("--sim-weak-joint must be an integer in [0,11]");
        }
        hardware_simulation.weak_joint = static_cast<int>(value);
      } else {throw std::invalid_argument("unknown option: " + option);}
    } else if (std::strcmp(argv[index], "--render-gpu") == 0) {
      if (index + 1 >= argc) {
        throw std::invalid_argument("usage: mymit_robot_user [--render-gpu nvidia|auto]");
      }
      render_gpu = argv[++index];
    } else {
      throw std::invalid_argument(
              "usage: mymit_robot_user [--render-gpu nvidia|auto]");
    }
  }

  if (std::strcmp(render_gpu, "nvidia") != 0 &&
    std::strcmp(render_gpu, "auto") != 0)
  {
    throw std::invalid_argument("render GPU must be nvidia or auto");
  }
  hardware_simulation.validate();
  hardware_simulation.algorithm_backend = algorithm_backend;
  hardware_simulation.algorithm_config_directory = algorithm_config_directory;
  if (hardware_options_used && !hardware_simulation.enabled) {
    throw std::invalid_argument("hardware simulation options require --hardware-sim");
  }
  return {
    MYMIT_ROBOT_DM1_SCENE_PATH, render_gpu,
    hardware_simulation, algorithm_backend, algorithm_config_directory};
}

/**
 * @brief 补齐图形桌面会话的运行时环境，避免 GLFW 找不到用户会话。
 *
 * 从 ROS 2、IDE 或 systemd 用户服务启动时，这些变量偶尔不会被继承。
 * 仅在变量缺失且当前用户的 systemd 会话目录存在时补齐，不覆盖用户配置。
 */
void configureDesktopSession()
{
  const std::string runtime_dir = "/run/user/" + std::to_string(::getuid());
  const std::filesystem::path runtime_path(runtime_dir);

  auto setDefault = [](const char * name, const std::string & value) {
      const char * current = std::getenv(name);
      if ((current == nullptr || current[0] == '\0') &&
        ::setenv(name, value.c_str(), 1) != 0)
      {
        throw std::runtime_error(std::string("failed to set ") + name);
      }
    };

  if (std::filesystem::is_directory(runtime_path)) {
    setDefault("XDG_RUNTIME_DIR", runtime_dir);
    const std::filesystem::path bus_path = runtime_path / "bus";
    if (std::filesystem::exists(bus_path)) {
      setDefault("DBUS_SESSION_BUS_ADDRESS", "unix:path=" + bus_path.string());
    }
  }

  const char * current_authority = std::getenv("XAUTHORITY");
  if (current_authority == nullptr || current_authority[0] == '\0') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      const std::filesystem::path authority = std::filesystem::path(home) / ".Xauthority";
      if (std::filesystem::is_regular_file(authority)) {
        setDefault("XAUTHORITY", authority.string());
      }
    }
  }
}

/** @brief 在混合显卡笔记本上让 GLFW/OpenGL 创建 NVIDIA 上下文。 */
void configureGpuRendering(const char * render_gpu)
{
  if (std::strcmp(render_gpu, "auto") == 0) {
    // 清理旧 shell 遗留的 PRIME 变量，让 GLX 按当前桌面默认设备选择。
    ::unsetenv("__NV_PRIME_RENDER_OFFLOAD");
    ::unsetenv("__GLX_VENDOR_LIBRARY_NAME");
    ::unsetenv("__VK_LAYER_NV_optimus");
    return;
  }
  // 这些变量必须在 GlfwAdapter 创建 OpenGL 上下文之前设置。第三个参数为 1，
  // 确保 launch 的默认 nvidia 策略不会被桌面会话中的旧变量意外覆盖。
  if (setenv("__NV_PRIME_RENDER_OFFLOAD", "1", 1) != 0 ||
    setenv("__GLX_VENDOR_LIBRARY_NAME", "nvidia", 1) != 0 ||
    setenv("__VK_LAYER_NV_optimus", "NVIDIA_only", 1) != 0)
  {
    throw std::runtime_error("failed to configure NVIDIA PRIME render offload");
  }
}
}

int main(int argc, char ** argv)
{
  if (argc == 2 && std::strcmp(argv[1], "--help") == 0) {
    std::printf(
      "Usage: mymit_robot_user [--hardware-sim] [--render-gpu auto|nvidia]\n"
      "  --algorithm current_mit|opendog  select one controller (default current_mit)\n"
      "  --algorithm-config DIR           OpenDog config/algorithm directory\n"
      "Hardware simulation (all actual hardware algorithms, simulated devices):\n"
      "  --calibration FILE              same calibration format as hardware_main\n"
      "  --torque-limit NM               same default as hardware_main: 88\n"
      "  --sim-command-delay-ms MS       default 1\n"
      "  --sim-feedback-delay-ms MS      default 1\n"
      "  --sim-imu-period-ms MS          default 2\n"
      "  --sim-imu-delay-ms MS           default 2\n"
      "  --sim-motor-time-constant-ms MS default 4\n"
      "  --sim-weak-joint INDEX          joint 0..11 in FR,FL,RR,RL order\n"
      "  --sim-output-scale SCALE       remaining motor output, 0..1\n"
      "  --sim-impairment-after-s S      seconds after first enable\n"
      "  --sim-drop-every N             fail every Nth MIT write; 0 disables\n"
      "Device timing/response defaults are unmeasured stress assumptions.\n");
    return 0;
  }
  try {
    const RobotSelection robot = selectRobot(argc, argv);
#if defined(MYMIT_ENABLE_OPENDOG) && !defined(MYMIT_OPENDOG_USER_STANDALONE)
    if (robot.algorithm_backend == AlgorithmBackendId::OpenDog) {
      // Keep the original MuJoCo executable's dependency order intact.  The
      // sibling OpenDog process loads ROS2's tinyxml2 before MuJoCo and then
      // runs the same main/SimulationBridge code with the selected algorithm.
      const std::filesystem::path self(argv[0] == nullptr ? "" : argv[0]);
      const std::filesystem::path sibling = self.has_parent_path() ?
        self.parent_path() / "mymit_opendog_user" :
        std::filesystem::path("mymit_opendog_user");
      std::string replacement = sibling.string();
      argv[0] = const_cast<char *>(replacement.c_str());
      if (self.has_parent_path()) {
        ::execv(replacement.c_str(), argv);
      } else {
        ::execvp(replacement.c_str(), argv);
      }
      throw std::runtime_error(
              "unable to dispatch OpenDog MuJoCo process: " + replacement);
    }
#elif defined(MYMIT_OPENDOG_USER_STANDALONE)
    if (robot.algorithm_backend != AlgorithmBackendId::OpenDog) {
      throw std::invalid_argument(
              "mymit_opendog_user only accepts --algorithm opendog");
    }
#endif
#if defined(MYMIT_ENABLE_OPENDOG)
    bool ros_initialized = false;
    if (robot.algorithm_backend == AlgorithmBackendId::OpenDog) {
      // The simulation parser already consumed the application arguments;
      // initialize ROS without handing custom --algorithm flags to rclcpp.
      rclcpp::init(0, nullptr);
      ros_initialized = true;
    }
#endif
    configureDesktopSession();
    configureGpuRendering(robot.render_gpu);
    SimulationBridge bridge(robot.scene_path, RobotType::DM1);
    bridge.setAlgorithm(robot.algorithm_backend, robot.algorithm_config_directory);
    bridge.setHardwareSimulation(robot.hardware_simulation);
    const int result = bridge.run();
#if defined(MYMIT_ENABLE_OPENDOG)
    if (ros_initialized) {rclcpp::shutdown();}
#endif
    return result;
  } catch (const std::exception & error) {
    std::fprintf(stderr, "simulation failed: %s\n", error.what());
    return 1;
  }
}
