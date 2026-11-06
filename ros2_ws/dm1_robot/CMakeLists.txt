cmake_minimum_required(VERSION 3.8)
project(dm1_robot)

# ROS 2 Humble 的 Python 生成工具安装在系统 Python 3.10 中。用户激活
# Conda 后，PATH 中的 Python 3.14 会被 CMake 优先选中，但该解释器既没有
# Humble 所需的 Empy/numpy，也不与发行版 ABI 匹配。统一两个新旧 CMake
# 查找变量，并保留 DM1_ROS_PYTHON_EXECUTABLE 供非标准安装显式覆盖。
set(DM1_ROS_PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH
  "Python interpreter matching the installed ROS 2 distribution")
if(EXISTS "${DM1_ROS_PYTHON_EXECUTABLE}")
  set(Python3_EXECUTABLE "${DM1_ROS_PYTHON_EXECUTABLE}" CACHE FILEPATH
    "Python 3 interpreter used by ament" FORCE)
  set(PYTHON_EXECUTABLE "${DM1_ROS_PYTHON_EXECUTABLE}" CACHE FILEPATH
    "Python interpreter used by rosidl_generator_py" FORCE)
else()
  message(FATAL_ERROR
    "DM1_ROS_PYTHON_EXECUTABLE does not exist: ${DM1_ROS_PYTHON_EXECUTABLE}")
endif()

# 使用 GLVND 而不是固定绑定 Mesa 的传统 libGL，运行时才能通过 PRIME 在
# Intel 与 NVIDIA OpenGL 实现之间选择。
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()
set(OpenGL_GL_PREFERENCE GLVND)

# OpenDog MPC/WBC 每个物理周期都会执行大量 Eigen 矩阵运算。未指定构建类型时，CMake
# 默认不启用优化，会让交互仿真明显低于实时速度；默认使用 Release，同时仍允许
# 调用者通过 --cmake-args -DCMAKE_BUILD_TYPE=... 显式覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(OpenGL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# OpenDog WBC+NMPC is the only controller. Its OCS2 numerical framework is
# linked from the sibling OpenDog ROS 2 install and overlay.
set(DM1_OPENDOG_PREFIX "/home/simon/open-dog/ros2_ws/install" CACHE PATH
  "OpenDog ROS 2 install prefix")
set(DM1_OPENDOG_OVERLAY "/home/simon/open-dog/.ros2_overlay/opt/ros/humble" CACHE PATH
  "OpenDog ROS 2 overlay prefix")
if(NOT EXISTS "${DM1_OPENDOG_PREFIX}/lib/libocs2_sqp.a" OR
  NOT EXISTS "${DM1_OPENDOG_PREFIX}/lib/libocs2_mpc.a")
  message(FATAL_ERROR
    "OpenDog OCS2 install not found under DM1_OPENDOG_PREFIX=${DM1_OPENDOG_PREFIX}")
endif()

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenMP REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(urdfdom_headers REQUIRED)

ament_export_dependencies(Eigen3)
ament_export_dependencies(mujoco_vendor)
ament_export_include_directories(include)

# 安装供其他节点和功能包使用的公共头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装 DM1 模型、启动文件和可追溯的模型契约。
install(DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(PROGRAMS scripts/setup_dm1_can.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Use one MuJoCo ABI for the sensor library, tests, and GUI. The local
# 3.11 installation also provides the Simulate UI used by sim_main.
set(DM1_LOCAL_MUJOCO_INCLUDE "${CMAKE_CURRENT_BINARY_DIR}/local_mujoco_include")
file(MAKE_DIRECTORY "${DM1_LOCAL_MUJOCO_INCLUDE}")
if(NOT EXISTS "${DM1_LOCAL_MUJOCO_INCLUDE}/mujoco")
  file(CREATE_LINK
    /usr/local/include/mujoco
    "${DM1_LOCAL_MUJOCO_INCLUDE}/mujoco"
    SYMBOLIC
  )
endif()

# DM1 IMU / 关节反馈数据源（MuJoCo 仿真与真实硬件共用的读取接口）。
add_library(dm1_robot_sensor
  src/sensor/imu.cpp
  src/sensor/imu_driver.cpp
  src/sensor/hfi_imu_driver.cpp
  src/sensor/leg.cpp
  third_party/dm_imu/src/bsp_crc.cpp
)
target_compile_features(dm1_robot_sensor PUBLIC cxx_std_17)
target_include_directories(dm1_robot_sensor BEFORE PRIVATE
  "${DM1_LOCAL_MUJOCO_INCLUDE}"
)
target_include_directories(dm1_robot_sensor PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/dm_imu/include"
)
target_include_directories(dm1_robot_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dm1_robot_sensor
  Eigen3::Eigen
  /usr/local/lib/libmujoco.so
)
install(TARGETS dm1_robot_sensor
  EXPORT dm1_robot_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Terminal input and the shared motor-output arbiter are kept separate from
# the sensor library so POSIX TTY/thread code does not leak into
# sensor-only consumers and tests.
add_library(dm1_robot_teleop
  src/teleop/operator_command.cpp
  src/teleop/keyboard_teleop.cpp
)
target_compile_features(dm1_robot_teleop PUBLIC cxx_std_17)
target_include_directories(dm1_robot_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dm1_robot_teleop PUBLIC pthread)
install(TARGETS dm1_robot_teleop
  EXPORT dm1_robot_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# The DM protocol is compiled as a small internal, ROS-independent library.
# The legacy ROS 1 legged_damiao_hw package is intentionally not part of this
# target or its dependency graph.
add_library(dm1_dmbot_serial
  third_party/dmbot_serial/src/protocol/socketcan.cpp
)
target_compile_features(dm1_dmbot_serial PUBLIC cxx_std_17)
target_include_directories(dm1_dmbot_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/dmbot_serial/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dm1_dmbot_serial PUBLIC pthread)
install(TARGETS dm1_dmbot_serial
  EXPORT dm1_robot_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Source-level OpenDog algorithm components. Only the WBC, legged OCP/reference
# manager, and Kalman estimator are copied here. Gazebo, simulation,
# controller_manager, and hardware plugins are intentionally excluded.
# OCS2 remains a linked numerical framework dependency.
set(DM1_OPENDOG_INCLUDE_DIRS
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_wbc/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_interface/include"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_estimation_ros2/include"
  "${DM1_OPENDOG_PREFIX}/include"
  "${DM1_OPENDOG_OVERLAY}/include"
  "/opt/ros/humble/include/urdfdom_headers"
  "/opt/ros/humble/include/urdfdom"
)
set(DM1_OPENDOG_LINK_DIRS
  "${DM1_OPENDOG_PREFIX}/lib"
  "${DM1_OPENDOG_OVERLAY}/lib/x86_64-linux-gnu"
  "${DM1_OPENDOG_OVERLAY}/../../../usr/lib/x86_64-linux-gnu"
)
set(DM1_OPENDOG_COMPILE_DEFINITIONS
  BOOST_MPL_LIMIT_LIST_SIZE=30
  BOOST_MPL_LIMIT_VECTOR_SIZE=30
  BOOST_MPL_LIMIT_MAP_SIZE=30
  BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
)

# The OCS2 static archives reference each other. Resolve them as a group,
# then link their shared numerical dependencies after the archives.
add_library(dm1_opendog_numerics INTERFACE)
target_link_directories(dm1_opendog_numerics INTERFACE ${DM1_OPENDOG_LINK_DIRS})
target_link_libraries(dm1_opendog_numerics INTERFACE
  "-Wl,--start-group"
  dm1_opendog_sqp dm1_opendog_mpc ocs2_qp_solver ocs2_oc ocs2_ddp ocs2_ipm
  ocs2_legged_robot ocs2_self_collision ocs2_centroidal_model
  ocs2_pinocchio_interface ocs2_robotic_tools ocs2_core
  "-Wl,--end-group"
  dm1_opendog_hpipm hpipm qpOASES pinocchio_default pinocchio_parsers
  hpp-fcl coal tinyxml2::tinyxml2 Boost::filesystem Boost::system
  ${OpenMP_CXX_LIBRARIES} pthread dl)

add_library(dm1_opendog_hpipm
  third_party/opendog/hpipm_interface/src/HpipmInterface.cpp
  third_party/opendog/hpipm_interface/src/HpipmInterfaceSettings.cpp)
target_compile_features(dm1_opendog_hpipm PUBLIC cxx_std_17)
target_include_directories(dm1_opendog_hpipm PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/hpipm_interface/include>
  $<BUILD_INTERFACE:${DM1_OPENDOG_PREFIX}/include>
  $<INSTALL_INTERFACE:include>)
target_link_directories(dm1_opendog_hpipm PUBLIC ${DM1_OPENDOG_LINK_DIRS})
target_link_libraries(dm1_opendog_hpipm PUBLIC Eigen3::Eigen
  "-Wl,--start-group" ocs2_qp_solver ocs2_oc ocs2_core "-Wl,--end-group"
  hpipm blasfeo Boost::filesystem Boost::system ${OpenMP_CXX_LIBRARIES} pthread dl)

add_library(dm1_opendog_mpc
  third_party/opendog/ocs2_mpc/src/MPC_BASE.cpp
  third_party/opendog/ocs2_mpc/src/MPC_Settings.cpp
  third_party/opendog/ocs2_mpc/src/SystemObservation.cpp
  third_party/opendog/ocs2_mpc/src/MRT_BASE.cpp
  third_party/opendog/ocs2_mpc/src/MPC_MRT_Interface.cpp)
target_compile_features(dm1_opendog_mpc PRIVATE cxx_std_17)
target_include_directories(dm1_opendog_mpc PRIVATE
  third_party/opendog/ocs2_mpc/include "${DM1_OPENDOG_PREFIX}/include")
target_link_directories(dm1_opendog_mpc PRIVATE ${DM1_OPENDOG_LINK_DIRS})
target_link_libraries(dm1_opendog_mpc PRIVATE Eigen3::Eigen
  "-Wl,--start-group" ocs2_oc ocs2_core "-Wl,--end-group"
  Boost::filesystem Boost::system ${OpenMP_CXX_LIBRARIES} pthread dl)

add_library(dm1_opendog_sqp
  third_party/opendog/ocs2_sqp/src/SqpLogging.cpp
  third_party/opendog/ocs2_sqp/src/SqpSettings.cpp
  third_party/opendog/ocs2_sqp/src/SqpSolver.cpp)
target_compile_features(dm1_opendog_sqp PRIVATE cxx_std_17)
target_include_directories(dm1_opendog_sqp PRIVATE
  third_party/opendog/ocs2_sqp/include third_party/opendog/ocs2_mpc/include third_party/opendog/hpipm_interface/include
  "${DM1_OPENDOG_PREFIX}/include")
target_link_directories(dm1_opendog_sqp PRIVATE ${DM1_OPENDOG_LINK_DIRS})
target_link_libraries(dm1_opendog_sqp PRIVATE Eigen3::Eigen
  dm1_opendog_mpc dm1_opendog_hpipm
  "-Wl,--start-group" ocs2_qp_solver ocs2_oc ocs2_core "-Wl,--end-group"
  Boost::filesystem Boost::system ${OpenMP_CXX_LIBRARIES} pthread dl)

add_library(dm1_opendog_wbc
  third_party/opendog/legged_wbc/src/HoQp.cpp
  third_party/opendog/legged_wbc/src/WbcBase.cpp
  third_party/opendog/legged_wbc/src/HierarchicalWbc.cpp
  third_party/opendog/legged_wbc/src/WeightedWbc.cpp
)
target_compile_features(dm1_opendog_wbc PUBLIC cxx_std_17)
target_compile_definitions(dm1_opendog_wbc PUBLIC ${DM1_OPENDOG_COMPILE_DEFINITIONS})
target_include_directories(dm1_opendog_wbc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_wbc/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_estimation_ros2/include>
  $<BUILD_INTERFACE:${DM1_OPENDOG_PREFIX}/include>
  $<BUILD_INTERFACE:${DM1_OPENDOG_OVERLAY}/include>
  $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom_headers>
  $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom>
  $<INSTALL_INTERFACE:include>)
target_link_directories(dm1_opendog_wbc PUBLIC ${DM1_OPENDOG_LINK_DIRS})
target_link_libraries(dm1_opendog_wbc
  Eigen3::Eigen dm1_opendog_numerics)

add_library(dm1_opendog_interface
  third_party/opendog/legged_interface/src/constraint/EndEffectorLinearConstraint.cpp
  third_party/opendog/legged_interface/src/constraint/FrictionConeConstraint.cpp
  third_party/opendog/legged_interface/src/constraint/NormalVelocityConstraintCppAd.cpp
  third_party/opendog/legged_interface/src/constraint/SwingTrajectoryPlanner.cpp
  third_party/opendog/legged_interface/src/constraint/ZeroForceConstraint.cpp
  third_party/opendog/legged_interface/src/constraint/ZeroVelocityConstraintCppAd.cpp
  third_party/opendog/legged_interface/src/initialization/LeggedRobotInitializer.cpp
  third_party/opendog/legged_interface/src/SwitchedModelReferenceManager.cpp
  third_party/opendog/legged_interface/src/LeggedRobotPreComputation.cpp
  third_party/opendog/legged_interface/src/LeggedInterface.cpp
)
target_compile_features(dm1_opendog_interface PUBLIC cxx_std_17)
target_compile_definitions(dm1_opendog_interface PUBLIC ${DM1_OPENDOG_COMPILE_DEFINITIONS})
target_include_directories(dm1_opendog_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_wbc/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_estimation_ros2/include>
  $<BUILD_INTERFACE:${DM1_OPENDOG_PREFIX}/include>
  $<BUILD_INTERFACE:${DM1_OPENDOG_OVERLAY}/include>
  $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom_headers>
  $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom>
  $<INSTALL_INTERFACE:include>)
target_link_directories(dm1_opendog_interface PUBLIC ${DM1_OPENDOG_LINK_DIRS})
target_link_libraries(dm1_opendog_interface PUBLIC
  Eigen3::Eigen dm1_opendog_numerics)
ament_target_dependencies(dm1_opendog_interface PUBLIC
  rcl rclcpp rclcpp_lifecycle std_msgs urdfdom_headers)

add_library(dm1_opendog_estimation
  third_party/opendog/legged_estimation_ros2/src/StateEstimateBase.cpp
  third_party/opendog/legged_estimation_ros2/src/LinearKalmanFilter.cpp
  third_party/opendog/legged_estimation_ros2/src/FromTopicEstimate.cpp
)
target_compile_features(dm1_opendog_estimation PUBLIC cxx_std_17)
target_compile_definitions(dm1_opendog_estimation PUBLIC ${DM1_OPENDOG_COMPILE_DEFINITIONS})
target_include_directories(dm1_opendog_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_wbc/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_estimation_ros2/include>
  $<BUILD_INTERFACE:${DM1_OPENDOG_PREFIX}/include>
  $<BUILD_INTERFACE:${DM1_OPENDOG_OVERLAY}/include>
  $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom_headers>
  $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom>
  $<INSTALL_INTERFACE:include>)
target_link_directories(dm1_opendog_estimation PUBLIC ${DM1_OPENDOG_LINK_DIRS})
target_link_libraries(dm1_opendog_estimation PUBLIC
  Eigen3::Eigen dm1_opendog_numerics)
ament_target_dependencies(dm1_opendog_estimation PUBLIC
  rcl rclcpp geometry_msgs nav_msgs realtime_tools
  tf2 tf2_geometry_msgs tf2_ros urdfdom_headers)

add_library(dm1_opendog_algorithm
  src/control/opendog_algorithm.cpp
  src/control/opendog_reference.cpp)
target_compile_features(dm1_opendog_algorithm PUBLIC cxx_std_17)
target_compile_definitions(dm1_opendog_algorithm PUBLIC ${DM1_OPENDOG_COMPILE_DEFINITIONS})
target_include_directories(dm1_opendog_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_wbc/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/opendog/legged_estimation_ros2/include>
  $<BUILD_INTERFACE:${DM1_OPENDOG_PREFIX}/include>
  $<BUILD_INTERFACE:${DM1_OPENDOG_OVERLAY}/include>
  $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom_headers>
  $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom>
  $<INSTALL_INTERFACE:include>)
target_link_directories(dm1_opendog_algorithm PUBLIC ${DM1_OPENDOG_LINK_DIRS})
target_link_libraries(dm1_opendog_algorithm PUBLIC
  Eigen3::Eigen
  dm1_opendog_wbc dm1_opendog_interface dm1_opendog_estimation
  dm1_opendog_numerics)
ament_target_dependencies(dm1_opendog_algorithm PUBLIC rclcpp)

install(TARGETS dm1_opendog_numerics dm1_opendog_hpipm dm1_opendog_mpc dm1_opendog_sqp
  dm1_opendog_wbc dm1_opendog_interface dm1_opendog_estimation
  dm1_opendog_algorithm
  EXPORT dm1_robot_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY third_party/opendog/legged_wbc/include/ DESTINATION include)
install(DIRECTORY third_party/opendog/legged_interface/include/ DESTINATION include)
install(DIRECTORY third_party/opendog/legged_estimation_ros2/include/ DESTINATION include)
install(DIRECTORY third_party/opendog/hpipm_interface/include/ DESTINATION include)
install(DIRECTORY third_party/opendog/ocs2_mpc/include/ DESTINATION include)
install(DIRECTORY third_party/opendog/ocs2_sqp/include/ DESTINATION include)

ament_export_targets(dm1_robot_targets HAS_LIBRARY_TARGET)
ament_export_libraries(dm1_robot_sensor)

# Formal DM1 hardware layer.  The concrete Dm1HardwareIo implementation and
# the control bridge live in this target; tests inject only a small mock for
# failure-path coverage instead of becoming the owner of the interface.
add_library(dm1_opendog_hardware_runner src/control/opendog_robot_runner.cpp)
target_compile_features(dm1_opendog_hardware_runner PUBLIC cxx_std_17)
target_link_libraries(dm1_opendog_hardware_runner PUBLIC
  dm1_opendog_algorithm dm1_robot_teleop)
install(TARGETS dm1_opendog_hardware_runner
  EXPORT dm1_robot_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)

add_library(dm1_robot_hardware
  src/hardware/hardware_bridge.cpp
  src/hardware/dm1_hardware_driver.cpp
  src/hardware/dm_motor_driver.cpp
)
target_compile_features(dm1_robot_hardware PUBLIC cxx_std_17)
target_include_directories(dm1_robot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/dmbot_serial/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(dm1_robot_hardware PRIVATE
  BOOST_MPL_LIMIT_LIST_SIZE=30
  BOOST_MPL_LIMIT_VECTOR_SIZE=30
  BOOST_MPL_LIMIT_MAP_SIZE=30
  BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
target_include_directories(dm1_robot_hardware PRIVATE ${DM1_OPENDOG_INCLUDE_DIRS})
target_link_directories(dm1_robot_hardware PUBLIC ${DM1_OPENDOG_LINK_DIRS})
target_link_libraries(dm1_robot_hardware PUBLIC
  # MuJoCo exports its bundled tinyxml2 symbols globally.  Keep ROS 2's
  # ABI-compatible system tinyxml2 in the hardware process dependency order
  # so rclcpp/fastdds resolve the same library as the rest of ROS 2.
  tinyxml2::tinyxml2
  dm1_opendog_hardware_runner
  dm1_robot_sensor
  dm1_robot_teleop
  dm1_dmbot_serial
  Eigen3::Eigen
  ${OpenMP_CXX_LIBRARIES}
  pthread
)
# tinyxml2 is otherwise dropped by --as-needed because no local object calls
# it directly; retain it as a DT_NEEDED entry before libmujoco is loaded.
target_link_options(dm1_robot_hardware PRIVATE -Wl,--no-as-needed)
ament_target_dependencies(dm1_robot_hardware PUBLIC rclcpp)
install(TARGETS dm1_robot_hardware
  EXPORT dm1_robot_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# MuJoCo GUI application. main only starts SimulationBridge, which runs the
# OpenDog estimator -> SQP-MPC -> WBC pipeline (or the full HardwareBridge with
# --hardware-sim). MuJoCo and ROS 2 ship incompatible global tinyxml2 symbol
# providers, so system tinyxml2 is deliberately linked first.
add_executable(sim_main
  src/app/sim_main.cpp
  src/simulation/HardwareSimulation.cpp
  src/simulation/SimulationActuatorWriter.cpp
  src/simulation/SimulationBridge.cpp
)
target_compile_definitions(sim_main PRIVATE
  DM1_ROBOT_DM1_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/description/scene.xml"
  DM1_ROBOT_DM1_CALIBRATION_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/hardware/dm1_hardware_calibration.txt"
  DM1_ROBOT_ALGORITHM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/opendog"
)
target_include_directories(sim_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/dm_imu/include
)
target_include_directories(sim_main BEFORE PRIVATE
  "${DM1_LOCAL_MUJOCO_INCLUDE}"
)
target_include_directories(sim_main SYSTEM PRIVATE
  /usr/local/include/simulate
)
target_link_libraries(sim_main
  tinyxml2::tinyxml2
  dm1_robot_hardware
  dm1_robot_sensor
  dm1_robot_teleop
  dm1_dmbot_serial
  Eigen3::Eigen
  /usr/local/lib/libmujoco.so
  /usr/local/lib/libsimulate.a
  OpenGL::OpenGL
  OpenGL::GLX
  glfw
  dl
  pthread
  util
)
target_link_options(sim_main PRIVATE -Wl,--no-as-needed)
ament_target_dependencies(sim_main rclcpp)
# libsimulate.a is distributed with GCC LTO objects. Tell GCC to use the
# available parallelism when materializing those objects; otherwise lto-wrapper
# falls back to serial LTRANS and emits a warning on every clean link.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_options(sim_main PRIVATE -flto=auto)
endif()

# DM1 real-hardware executable. The vendor CAN/IMU calls are intentionally
# isolated in src/hardware and default to fail-closed until connected.
add_executable(hardware_main
  src/app/hardware_main.cpp
)
target_compile_definitions(hardware_main PRIVATE
  DM1_ROBOT_ALGORITHM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/opendog"
)
target_compile_features(hardware_main PRIVATE cxx_std_17)
target_include_directories(hardware_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(hardware_main BEFORE PRIVATE
  "${DM1_LOCAL_MUJOCO_INCLUDE}"
)
target_link_libraries(hardware_main
  tinyxml2::tinyxml2
  dm1_robot_hardware
)
target_link_options(hardware_main PRIVATE -Wl,--no-as-needed)
ament_target_dependencies(hardware_main rclcpp)

# Preserve the OpenDog dependency paths after installation as well as in build/.
set_target_properties(dm1_opendog_hpipm dm1_opendog_mpc dm1_opendog_sqp
  dm1_opendog_wbc dm1_opendog_interface dm1_opendog_estimation
  dm1_opendog_algorithm dm1_opendog_hardware_runner
  dm1_robot_hardware sim_main hardware_main
  PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS sim_main hardware_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_opendog_algorithm
    test/control/opendog_algorithm_test.cpp test/control/opendog_reference_test.cpp)
  target_link_libraries(test_opendog_algorithm dm1_opendog_algorithm)
  # libhpipm.so has no RUNPATH for its blasfeo dependency; keep blasfeo as a
  # direct DT_NEEDED entry so the test resolves it through its own RUNPATH.
  target_link_options(test_opendog_algorithm PRIVATE -Wl,--no-as-needed)
  target_compile_definitions(test_opendog_algorithm PRIVATE
    DM1_OPENDOG_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/opendog")
  # Three closed-loop MuJoCo runs; the first run also generates CppAD libraries.
  ament_add_gtest(test_opendog_mujoco
    test/control/opendog_mujoco_test.cpp src/simulation/SimulationActuatorWriter.cpp TIMEOUT 300)
  target_compile_definitions(test_opendog_mujoco PRIVATE
    DM1_OPENDOG_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/opendog"
    DM1_ROBOT_TEST_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/description/scene.xml")
  target_include_directories(test_opendog_mujoco PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_opendog_mujoco
    tinyxml2::tinyxml2 dm1_opendog_algorithm dm1_robot_sensor)
  target_link_options(test_opendog_mujoco PRIVATE -Wl,--no-as-needed)
  ament_target_dependencies(test_opendog_mujoco rclcpp)
  add_test(NAME test_opendog_model_contract
    COMMAND ${DM1_ROS_PYTHON_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/model/opendog_model_contract_test.py)
  find_package(ament_lint_auto REQUIRED)
  add_subdirectory(test)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  # The repository also carries archived ROS 1/vendor sources which are not
  # part of this package's targets. Keep lint focused on the active ROS 2
  # hardware path instead of rewriting those external snapshots.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_lint_cmake
    ament_cmake_uncrustify
    ament_cmake_xmllint
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_lint_cmake(CMakeLists.txt test/CMakeLists.txt)
  ament_uncrustify(
    third_party/dm_imu/include
    third_party/dm_imu/src/bsp_crc.cpp
    third_party/dmbot_serial/include/dmbot_serial/protocol/socketcan.h
    third_party/dmbot_serial/include/dmbot_serial/protocol/terminal_colors.h
    third_party/dmbot_serial/src/protocol/socketcan.cpp
    include/common
    include/hardware
    include/teleop
    include/sensor
    src/hardware
    src/sensor
    src/teleop
    test/teleop
    test/hardware
  )
  ament_xmllint(
    description/dm1.xml
    description/scene.xml
  )
endif()

ament_package()
