cmake_minimum_required(VERSION 3.8)
project(mymit_robot)

# ROS 2 Humble 的 Python 生成工具安装在系统 Python 3.10 中。用户激活
# Conda 后，PATH 中的 Python 3.14 会被 CMake 优先选中，但该解释器既没有
# Humble 所需的 Empy/numpy，也不与发行版 ABI 匹配。统一两个新旧 CMake
# 查找变量，并保留 MYMIT_ROS_PYTHON_EXECUTABLE 供非标准安装显式覆盖。
set(MYMIT_ROS_PYTHON_EXECUTABLE "/usr/bin/python3" CACHE FILEPATH
  "Python interpreter matching the installed ROS 2 distribution")
if(EXISTS "${MYMIT_ROS_PYTHON_EXECUTABLE}")
  set(Python3_EXECUTABLE "${MYMIT_ROS_PYTHON_EXECUTABLE}" CACHE FILEPATH
    "Python 3 interpreter used by ament" FORCE)
  set(PYTHON_EXECUTABLE "${MYMIT_ROS_PYTHON_EXECUTABLE}" CACHE FILEPATH
    "Python interpreter used by rosidl_generator_py" FORCE)
else()
  message(FATAL_ERROR
    "MYMIT_ROS_PYTHON_EXECUTABLE does not exist: ${MYMIT_ROS_PYTHON_EXECUTABLE}")
endif()

# 使用 GLVND 而不是固定绑定 Mesa 的传统 libGL，运行时才能通过 PRIME 在
# Intel 与 NVIDIA OpenGL 实现之间选择。
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()
set(OpenGL_GL_PREFERENCE GLVND)

# MPC/WBC 每个物理周期都会执行大量 Eigen 矩阵运算。未指定构建类型时，CMake
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

# qpOASES is vendored from the MIT Cheetah third-party tree so the controller
# uses the same online active-set QP backend without relying on a system install.
file(GLOB MYMIT_QPOASES_SOURCES CONFIGURE_DEPENDS
  third_party/qpOASES/src/*.cpp
)
find_package(geometry_msgs REQUIRED)
find_package(mujoco_vendor REQUIRED)
find_package(OpenGL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# OpenDog is an optional second backend.  Its source files stay separate from
# the current MIT controller and are compiled against the existing ROS 2
# install when the sibling OpenDog workspace is available.
set(MYMIT_OPENDOG_PREFIX "/home/simon/open-dog/ros2_ws/install" CACHE PATH
  "OpenDog ROS 2 install prefix")
set(MYMIT_OPENDOG_OVERLAY "/home/simon/open-dog/.ros2_overlay/opt/ros/humble" CACHE PATH
  "OpenDog ROS 2 overlay prefix")
option(MYMIT_ENABLE_OPENDOG "Build the OpenDog WBC+NMPC backend" ON)
if(MYMIT_ENABLE_OPENDOG AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_interface/include/legged_interface/LeggedInterface.h" AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_wbc/src/WeightedWbc.cpp" AND
  EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_estimation_ros2/src/LinearKalmanFilter.cpp" AND
  EXISTS "${MYMIT_OPENDOG_PREFIX}/lib/libocs2_sqp.a" AND
  EXISTS "${MYMIT_OPENDOG_PREFIX}/lib/libocs2_mpc.a")
  set(MYMIT_OPENDOG_AVAILABLE TRUE)
else()
  set(MYMIT_OPENDOG_AVAILABLE FALSE)
  if(MYMIT_ENABLE_OPENDOG)
    message(WARNING "OpenDog install not found; building the current MIT backend only")
  endif()
endif()

if(MYMIT_OPENDOG_AVAILABLE)
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
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootContact.msg"
  "msg/StateEstimate.msg"
  "srv/SetControlMode.srv"
  "srv/SetStandingHeight.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(Eigen3)
ament_export_dependencies(mujoco_vendor)
ament_export_include_directories(include)

# 安装供其他节点和功能包使用的公共头文件
install(DIRECTORY include/
  DESTINATION include
  PATTERN "MPC/common_types.h" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/BodyOriTask.hpp" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/BodyPosTask.hpp" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/BodyPostureTask.hpp" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/BodyRyRzTask.hpp" EXCLUDE
  PATTERN "WBC/WBC_Ctrl/SingleContact.hpp" EXCLUDE
)

# 安装 DM1 模型、启动文件和可追溯的模型契约。
install(DIRECTORY src/dm1_description/dm1
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY src/dm1_description/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(PROGRAMS scripts/setup_dm1_can.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Use one MuJoCo ABI for the model library, sensors, tests, and GUI. The local
# 3.11 installation also provides the Simulate UI used by mymit_robot_user.
set(MYMIT_LOCAL_MUJOCO_INCLUDE "${CMAKE_CURRENT_BINARY_DIR}/local_mujoco_include")
file(MAKE_DIRECTORY "${MYMIT_LOCAL_MUJOCO_INCLUDE}")
if(NOT EXISTS "${MYMIT_LOCAL_MUJOCO_INCLUDE}/mujoco")
  file(CREATE_LINK
    /usr/local/include/mujoco
    "${MYMIT_LOCAL_MUJOCO_INCLUDE}/mujoco"
    SYMBOLIC
  )
endif()

# 与 ROS 2/MuJoCo 解耦的 DM1 数学模型参数库
add_library(mymit_robot_model
  src/MPC/ConvexMPCLocomotion.cpp
  src/MPC/Gait.cpp
  src/MPC/RobotState.cpp
  src/MPC/SolverMPC.cpp
  ${MYMIT_QPOASES_SOURCES}
  src/FSM/ControlFSM.cpp
  src/FSM/FSM_State_BalanceStand.cpp
  src/FSM/FSM_State_LieDown.cpp
  src/FSM/FSM_State_Locomotion.cpp
  src/FSM/FSM_State_Passive.cpp
  src/FSM/FSM_State_RecoveryStand.cpp
  src/FSM/FSM_State_StandUp.cpp
  src/FSM/SafetyChecker.cpp
  src/WBC/FloatingBaseModel.cpp
  src/WBC/KinWBC.cpp
  src/WBC/LocomotionCtrl/LocomotionCtrl.cpp
  src/WBC/WBC_Ctrl/BodyOriTask.cpp
  src/WBC/WBC_Ctrl/BodyPosTask.cpp
  src/WBC/WBC_Ctrl/LinkPosTask.cpp
  src/WBC/ContactSet/SingleContact.cpp
  src/WBC/WBC_Ctrl/WBC_Ctrl.cpp
  src/WBC/WBIC.cpp
  src/controller/ContactEstimator.cpp
  src/controller/FootSwingTrajectory.cpp
  src/controller/GaitScheduler.cpp
  src/controller/OrientationEstimator.cpp
  src/controller/PositionVelocityEstimator.cpp
  src/controller/leg_controller.cpp
  src/model/quadruped.cpp
  src/sensor/imu.cpp
  src/sensor/imu_driver.cpp
  src/sensor/leg.cpp
  Middlewares/dm_imu/src/bsp_crc.cpp
)
target_compile_features(mymit_robot_model PUBLIC cxx_std_17)
target_compile_definitions(mymit_robot_model PRIVATE __NO_COPYRIGHT__)
target_include_directories(mymit_robot_model BEFORE PRIVATE
  "${MYMIT_LOCAL_MUJOCO_INCLUDE}"
)
target_include_directories(mymit_robot_model PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
  "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/dm_imu/include"
)
target_include_directories(mymit_robot_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/qpOASES/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mymit_robot_model
  Eigen3::Eigen
  /usr/local/lib/libmujoco.so
)
install(TARGETS mymit_robot_model
  EXPORT mymit_robot_model_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Terminal input and the shared motor-output arbiter are kept separate from
# the mathematical model library so POSIX TTY/thread code does not leak into
# model-only consumers and tests.
add_library(mymit_robot_teleop
  src/teleop/operator_command.cpp
  src/teleop/keyboard_teleop.cpp
)
target_compile_features(mymit_robot_teleop PUBLIC cxx_std_17)
target_include_directories(mymit_robot_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mymit_robot_teleop PUBLIC pthread)
install(TARGETS mymit_robot_teleop
  EXPORT mymit_robot_model_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# The DM protocol is compiled as a small internal, ROS-independent library.
# The legacy ROS 1 legged_damiao_hw package is intentionally not part of this
# target or its dependency graph.
add_library(mymit_dmbot_serial
  Middlewares/dmbot_serial/src/protocol/socketcan.cpp
)
target_compile_features(mymit_dmbot_serial PUBLIC cxx_std_17)
target_include_directories(mymit_dmbot_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/dmbot_serial/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mymit_dmbot_serial PUBLIC pthread)
install(TARGETS mymit_dmbot_serial
  EXPORT mymit_robot_model_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Source-level OpenDog algorithm components. Only the WBC, legged OCP/reference
# manager, and Kalman estimator are copied here. Gazebo, simulation,
# controller_manager, and hardware plugins are intentionally excluded.
# OCS2 remains a linked numerical framework dependency.
if(MYMIT_OPENDOG_AVAILABLE)
  set(MYMIT_OPENDOG_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_wbc/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_interface/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_estimation_ros2/include"
    "${MYMIT_OPENDOG_PREFIX}/include"
    "${MYMIT_OPENDOG_OVERLAY}/include"
    "/opt/ros/humble/include/urdfdom_headers"
    "/opt/ros/humble/include/urdfdom"
  )
  set(MYMIT_OPENDOG_LINK_DIRS
    "${MYMIT_OPENDOG_PREFIX}/lib"
    "${MYMIT_OPENDOG_OVERLAY}/lib/x86_64-linux-gnu"
    "${MYMIT_OPENDOG_OVERLAY}/../../../usr/lib/x86_64-linux-gnu"
  )
  set(MYMIT_OPENDOG_COMPILE_DEFINITIONS
    MYMIT_ENABLE_OPENDOG
    BOOST_MPL_LIMIT_LIST_SIZE=30
    BOOST_MPL_LIMIT_VECTOR_SIZE=30
    BOOST_MPL_LIMIT_MAP_SIZE=30
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
  )

  # The OCS2 static archives reference each other. Resolve them as a group,
  # then link their shared numerical dependencies after the archives.
  add_library(mymit_opendog_numerics INTERFACE)
  target_link_directories(mymit_opendog_numerics INTERFACE ${MYMIT_OPENDOG_LINK_DIRS})
  target_link_libraries(mymit_opendog_numerics INTERFACE
    "-Wl,--start-group"
    mymit_opendog_sqp mymit_opendog_mpc ocs2_qp_solver ocs2_oc ocs2_ddp ocs2_ipm
    ocs2_legged_robot ocs2_self_collision ocs2_centroidal_model
    ocs2_pinocchio_interface ocs2_robotic_tools ocs2_core
    "-Wl,--end-group"
    mymit_opendog_hpipm hpipm qpOASES pinocchio_default pinocchio_parsers
    hpp-fcl coal tinyxml2::tinyxml2 Boost::filesystem Boost::system
    ${OpenMP_CXX_LIBRARIES} pthread dl)

  add_library(mymit_opendog_hpipm
    opendog/hpipm_interface/src/HpipmInterface.cpp
    opendog/hpipm_interface/src/HpipmInterfaceSettings.cpp)
  target_compile_features(mymit_opendog_hpipm PUBLIC cxx_std_17)
  target_include_directories(mymit_opendog_hpipm PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/hpipm_interface/include>
    $<BUILD_INTERFACE:${MYMIT_OPENDOG_PREFIX}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_directories(mymit_opendog_hpipm PUBLIC ${MYMIT_OPENDOG_LINK_DIRS})
  target_link_libraries(mymit_opendog_hpipm PUBLIC Eigen3::Eigen
    "-Wl,--start-group" ocs2_qp_solver ocs2_oc ocs2_core "-Wl,--end-group"
    hpipm blasfeo Boost::filesystem Boost::system ${OpenMP_CXX_LIBRARIES} pthread dl)

  add_library(mymit_opendog_mpc
    opendog/ocs2_mpc/src/MPC_BASE.cpp
    opendog/ocs2_mpc/src/MPC_Settings.cpp
    opendog/ocs2_mpc/src/SystemObservation.cpp
    opendog/ocs2_mpc/src/MRT_BASE.cpp
    opendog/ocs2_mpc/src/MPC_MRT_Interface.cpp)
  target_compile_features(mymit_opendog_mpc PRIVATE cxx_std_17)
  target_include_directories(mymit_opendog_mpc PRIVATE
    opendog/ocs2_mpc/include "${MYMIT_OPENDOG_PREFIX}/include")
  target_link_directories(mymit_opendog_mpc PRIVATE ${MYMIT_OPENDOG_LINK_DIRS})
  target_link_libraries(mymit_opendog_mpc PRIVATE Eigen3::Eigen
    "-Wl,--start-group" ocs2_oc ocs2_core "-Wl,--end-group"
    Boost::filesystem Boost::system ${OpenMP_CXX_LIBRARIES} pthread dl)

  add_library(mymit_opendog_sqp
    opendog/ocs2_sqp/src/SqpLogging.cpp
    opendog/ocs2_sqp/src/SqpSettings.cpp
    opendog/ocs2_sqp/src/SqpSolver.cpp)
  target_compile_features(mymit_opendog_sqp PRIVATE cxx_std_17)
  target_include_directories(mymit_opendog_sqp PRIVATE
    opendog/ocs2_sqp/include opendog/ocs2_mpc/include opendog/hpipm_interface/include
    "${MYMIT_OPENDOG_PREFIX}/include")
  target_link_directories(mymit_opendog_sqp PRIVATE ${MYMIT_OPENDOG_LINK_DIRS})
  target_link_libraries(mymit_opendog_sqp PRIVATE Eigen3::Eigen
    mymit_opendog_mpc mymit_opendog_hpipm
    "-Wl,--start-group" ocs2_qp_solver ocs2_oc ocs2_core "-Wl,--end-group"
    Boost::filesystem Boost::system ${OpenMP_CXX_LIBRARIES} pthread dl)

  add_library(mymit_opendog_wbc
    opendog/legged_wbc/src/HoQp.cpp
    opendog/legged_wbc/src/WbcBase.cpp
    opendog/legged_wbc/src/HierarchicalWbc.cpp
    opendog/legged_wbc/src/WeightedWbc.cpp
  )
  target_compile_features(mymit_opendog_wbc PUBLIC cxx_std_17)
  target_compile_definitions(mymit_opendog_wbc PUBLIC ${MYMIT_OPENDOG_COMPILE_DEFINITIONS})
  target_include_directories(mymit_opendog_wbc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_wbc/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_interface/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_estimation_ros2/include>
    $<BUILD_INTERFACE:${MYMIT_OPENDOG_PREFIX}/include>
    $<BUILD_INTERFACE:${MYMIT_OPENDOG_OVERLAY}/include>
    $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom_headers>
    $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom>
    $<INSTALL_INTERFACE:include>)
  target_link_directories(mymit_opendog_wbc PUBLIC ${MYMIT_OPENDOG_LINK_DIRS})
  target_link_libraries(mymit_opendog_wbc
    Eigen3::Eigen mymit_opendog_numerics)

  add_library(mymit_opendog_interface
    opendog/legged_interface/src/constraint/EndEffectorLinearConstraint.cpp
    opendog/legged_interface/src/constraint/FrictionConeConstraint.cpp
    opendog/legged_interface/src/constraint/NormalVelocityConstraintCppAd.cpp
    opendog/legged_interface/src/constraint/SwingTrajectoryPlanner.cpp
    opendog/legged_interface/src/constraint/ZeroForceConstraint.cpp
    opendog/legged_interface/src/constraint/ZeroVelocityConstraintCppAd.cpp
    opendog/legged_interface/src/initialization/LeggedRobotInitializer.cpp
    opendog/legged_interface/src/SwitchedModelReferenceManager.cpp
    opendog/legged_interface/src/LeggedRobotPreComputation.cpp
    opendog/legged_interface/src/LeggedInterface.cpp
  )
  target_compile_features(mymit_opendog_interface PUBLIC cxx_std_17)
  target_compile_definitions(mymit_opendog_interface PUBLIC ${MYMIT_OPENDOG_COMPILE_DEFINITIONS})
  target_include_directories(mymit_opendog_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_wbc/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_interface/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_estimation_ros2/include>
    $<BUILD_INTERFACE:${MYMIT_OPENDOG_PREFIX}/include>
    $<BUILD_INTERFACE:${MYMIT_OPENDOG_OVERLAY}/include>
    $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom_headers>
    $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom>
    $<INSTALL_INTERFACE:include>)
  target_link_directories(mymit_opendog_interface PUBLIC ${MYMIT_OPENDOG_LINK_DIRS})
  target_link_libraries(mymit_opendog_interface PUBLIC
    Eigen3::Eigen mymit_opendog_numerics)
  ament_target_dependencies(mymit_opendog_interface PUBLIC
    rcl rclcpp rclcpp_lifecycle std_msgs urdfdom_headers)

  add_library(mymit_opendog_estimation
    opendog/legged_estimation_ros2/src/StateEstimateBase.cpp
    opendog/legged_estimation_ros2/src/LinearKalmanFilter.cpp
    opendog/legged_estimation_ros2/src/FromTopicEstimate.cpp
  )
  target_compile_features(mymit_opendog_estimation PUBLIC cxx_std_17)
  target_compile_definitions(mymit_opendog_estimation PUBLIC ${MYMIT_OPENDOG_COMPILE_DEFINITIONS})
  target_include_directories(mymit_opendog_estimation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_wbc/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_interface/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_estimation_ros2/include>
    $<BUILD_INTERFACE:${MYMIT_OPENDOG_PREFIX}/include>
    $<BUILD_INTERFACE:${MYMIT_OPENDOG_OVERLAY}/include>
    $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom_headers>
    $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom>
    $<INSTALL_INTERFACE:include>)
  target_link_directories(mymit_opendog_estimation PUBLIC ${MYMIT_OPENDOG_LINK_DIRS})
  target_link_libraries(mymit_opendog_estimation PUBLIC
    Eigen3::Eigen mymit_opendog_numerics)
  ament_target_dependencies(mymit_opendog_estimation PUBLIC
    rcl rclcpp geometry_msgs nav_msgs realtime_tools
    tf2 tf2_geometry_msgs tf2_ros urdfdom_headers)

  add_library(mymit_opendog_algorithm
    src/control/opendog_algorithm.cpp
    src/control/opendog_reference.cpp)
  target_compile_features(mymit_opendog_algorithm PUBLIC cxx_std_17)
  target_compile_definitions(mymit_opendog_algorithm PUBLIC ${MYMIT_OPENDOG_COMPILE_DEFINITIONS})
  target_include_directories(mymit_opendog_algorithm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_wbc/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_interface/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/opendog/legged_estimation_ros2/include>
    $<BUILD_INTERFACE:${MYMIT_OPENDOG_PREFIX}/include>
    $<BUILD_INTERFACE:${MYMIT_OPENDOG_OVERLAY}/include>
    $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom_headers>
    $<BUILD_INTERFACE:/opt/ros/humble/include/urdfdom>
    $<INSTALL_INTERFACE:include>)
  target_link_directories(mymit_opendog_algorithm PUBLIC ${MYMIT_OPENDOG_LINK_DIRS})
  target_link_libraries(mymit_opendog_algorithm PUBLIC
    Eigen3::Eigen
    mymit_opendog_wbc mymit_opendog_interface mymit_opendog_estimation
    mymit_opendog_numerics)
  ament_target_dependencies(mymit_opendog_algorithm PUBLIC rclcpp)

  install(TARGETS mymit_opendog_numerics mymit_opendog_hpipm mymit_opendog_mpc mymit_opendog_sqp
    mymit_opendog_wbc mymit_opendog_interface mymit_opendog_estimation
    mymit_opendog_algorithm
    EXPORT mymit_robot_model_targets
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(DIRECTORY opendog/legged_wbc/include/ DESTINATION include)
  install(DIRECTORY opendog/legged_interface/include/ DESTINATION include)
  install(DIRECTORY opendog/legged_estimation_ros2/include/ DESTINATION include)
  install(DIRECTORY opendog/hpipm_interface/include/ DESTINATION include)
  install(DIRECTORY opendog/ocs2_mpc/include/ DESTINATION include)
  install(DIRECTORY opendog/ocs2_sqp/include/ DESTINATION include)
endif()

ament_export_targets(mymit_robot_model_targets HAS_LIBRARY_TARGET)
ament_export_libraries(mymit_robot_model)

# Formal DM1 hardware layer.  The concrete Dm1HardwareIo implementation and
# the control bridge live in this target; tests inject only a small mock for
# failure-path coverage instead of becoming the owner of the interface.
add_library(mymit_robot_hardware
  src/hardware/hardware_bridge.cpp
  src/control/current_mit_backend.cpp
  src/hardware/dm1_hardware_driver.cpp
  src/hardware/dm_motor_driver.cpp
  user/RobotRunner.cpp
)
target_compile_features(mymit_robot_hardware PUBLIC cxx_std_17)
target_include_directories(mymit_robot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/user>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/dmbot_serial/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mymit_robot_hardware PUBLIC
  mymit_robot_model
  mymit_robot_teleop
  mymit_dmbot_serial
  Eigen3::Eigen
  pthread
)
if(MYMIT_OPENDOG_AVAILABLE)
  target_compile_definitions(mymit_robot_hardware PRIVATE MYMIT_ENABLE_OPENDOG)
  target_compile_definitions(mymit_robot_hardware PRIVATE
    BOOST_MPL_LIMIT_LIST_SIZE=30
    BOOST_MPL_LIMIT_VECTOR_SIZE=30
    BOOST_MPL_LIMIT_MAP_SIZE=30
    BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS)
  target_include_directories(mymit_robot_hardware PRIVATE ${MYMIT_OPENDOG_INCLUDE_DIRS})
  target_link_directories(mymit_robot_hardware PUBLIC
    "${MYMIT_OPENDOG_PREFIX}/lib"
    "${MYMIT_OPENDOG_OVERLAY}/lib/x86_64-linux-gnu"
    "${MYMIT_OPENDOG_OVERLAY}/../../../usr/lib/x86_64-linux-gnu"
  )
  target_link_libraries(mymit_robot_hardware PUBLIC
    # MuJoCo exports its bundled tinyxml2 symbols globally.  Keep ROS 2's
    # ABI-compatible system tinyxml2 in the hardware process dependency order
    # so rclcpp/fastdds resolve the same library as the rest of ROS 2.
    tinyxml2::tinyxml2
    mymit_opendog_algorithm ${OpenMP_CXX_LIBRARIES})
  # tinyxml2 is otherwise dropped by --as-needed because no local object calls
  # it directly; retain it as a DT_NEEDED entry before libmujoco is loaded.
  target_link_options(mymit_robot_hardware PRIVATE -Wl,--no-as-needed)
  ament_target_dependencies(mymit_robot_hardware PUBLIC rclcpp)
endif()
install(TARGETS mymit_robot_hardware
  EXPORT mymit_robot_model_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# MuJoCo GUI application. main only starts SimulationBridge; RobotRunner owns
# the complete estimator -> FSM -> MPC/WBC -> LegController control pipeline.
add_executable(mymit_robot_user
  user/main.cpp
  user/HardwareSimulation.cpp
  user/SimulationActuatorWriter.cpp
  user/SimulationBridge.cpp
)
target_compile_definitions(mymit_robot_user PRIVATE
  MYMIT_ROBOT_DM1_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/dm1_description/dm1/scene.xml"
  MYMIT_ROBOT_DM1_CALIBRATION_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/dm1_hardware_calibration.txt"
  MYMIT_ROBOT_ALGORITHM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/algorithm"
)
target_include_directories(mymit_robot_user PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/user
  ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/dm_imu/include
)
target_include_directories(mymit_robot_user BEFORE PRIVATE
  "${MYMIT_LOCAL_MUJOCO_INCLUDE}"
)
target_include_directories(mymit_robot_user SYSTEM PRIVATE
  /usr/local/include/simulate
)
target_link_libraries(mymit_robot_user
  mymit_robot_hardware
  mymit_robot_model
  mymit_robot_teleop
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
if(MYMIT_OPENDOG_AVAILABLE)
  target_compile_definitions(mymit_robot_user PRIVATE MYMIT_ENABLE_OPENDOG)
  target_link_options(mymit_robot_user PRIVATE -Wl,--no-as-needed)
  ament_target_dependencies(mymit_robot_user rclcpp)

  # MuJoCo and ROS2 ship incompatible global tinyxml2 symbol providers.  The
  # original user executable keeps MuJoCo first; OpenDog runs in this sibling
  # executable with system tinyxml2 deliberately first.  main.cpp dispatches
  # --algorithm opendog to this process before either runtime is initialized.
  add_executable(mymit_opendog_user
    user/main.cpp
    user/HardwareSimulation.cpp
    user/SimulationActuatorWriter.cpp
    user/SimulationBridge.cpp
  )
  target_compile_definitions(mymit_opendog_user PRIVATE
    MYMIT_ENABLE_OPENDOG
    MYMIT_OPENDOG_USER_STANDALONE
    MYMIT_ROBOT_DM1_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/dm1_description/dm1/scene.xml"
    MYMIT_ROBOT_DM1_CALIBRATION_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/dm1_hardware_calibration.txt"
    MYMIT_ROBOT_ALGORITHM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/algorithm"
  )
  target_include_directories(mymit_opendog_user PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/user
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/dm_imu/include
  )
  target_include_directories(mymit_opendog_user BEFORE PRIVATE
    "${MYMIT_LOCAL_MUJOCO_INCLUDE}"
  )
  target_include_directories(mymit_opendog_user SYSTEM PRIVATE
    /usr/local/include/simulate
  )
  target_link_libraries(mymit_opendog_user
    tinyxml2::tinyxml2
    mymit_robot_hardware
    mymit_robot_model
    mymit_robot_teleop
    mymit_dmbot_serial
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
  target_link_options(mymit_opendog_user PRIVATE -Wl,--no-as-needed)
  ament_target_dependencies(mymit_opendog_user rclcpp)
endif()
# libsimulate.a is distributed with GCC LTO objects. Tell GCC to use the
# available parallelism when materializing those objects; otherwise lto-wrapper
# falls back to serial LTRANS and emits a warning on every clean link.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_options(mymit_robot_user PRIVATE -flto=auto)
endif()
add_executable(mymit_robot_height_service
  user/StandingHeightService.cpp
)
target_compile_features(mymit_robot_height_service PRIVATE cxx_std_17)
target_include_directories(mymit_robot_height_service PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/user
)
ament_target_dependencies(mymit_robot_height_service rclcpp)
rosidl_get_typesupport_target(mymit_robot_height_service_typesupport
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(mymit_robot_height_service
  "${mymit_robot_height_service_typesupport}"
)

# DM1 real-hardware executable. The vendor CAN/IMU calls are intentionally
# isolated in src/hardware and default to fail-closed until connected.
add_executable(hardware_main
  src/hardware/hardware_main.cpp
)
target_compile_definitions(hardware_main PRIVATE
  MYMIT_ROBOT_ALGORITHM_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/algorithm"
)
target_compile_features(hardware_main PRIVATE cxx_std_17)
target_include_directories(hardware_main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/user
)
target_include_directories(hardware_main BEFORE PRIVATE
  "${MYMIT_LOCAL_MUJOCO_INCLUDE}"
)
if(MYMIT_OPENDOG_AVAILABLE)
  target_link_libraries(hardware_main tinyxml2::tinyxml2)
endif()
target_link_libraries(hardware_main
  mymit_robot_hardware
)
if(MYMIT_OPENDOG_AVAILABLE)
  target_link_options(hardware_main PRIVATE -Wl,--no-as-needed)
  target_compile_definitions(hardware_main PRIVATE MYMIT_ENABLE_OPENDOG)
  ament_target_dependencies(hardware_main rclcpp)
endif()

set(MYMIT_USER_RUNTIME_TARGETS mymit_robot_user mymit_robot_height_service hardware_main)
if(MYMIT_OPENDOG_AVAILABLE)
  list(APPEND MYMIT_USER_RUNTIME_TARGETS mymit_opendog_user)
endif()
# Preserve the OpenDog dependency paths after installation as well as in build/.
# Limit this to targets that load the OpenDog runtime; keep the MIT GUI link order.
if(MYMIT_OPENDOG_AVAILABLE)
  set_target_properties(mymit_opendog_hpipm mymit_opendog_mpc mymit_opendog_sqp
    mymit_opendog_wbc mymit_opendog_interface mymit_opendog_estimation
    mymit_opendog_algorithm mymit_robot_hardware mymit_opendog_user hardware_main
    PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()
install(TARGETS ${MYMIT_USER_RUNTIME_TARGETS}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  if(MYMIT_OPENDOG_AVAILABLE)
    ament_add_gtest(test_opendog_algorithm
      test/control/opendog_algorithm_test.cpp test/control/opendog_reference_test.cpp)
    target_link_libraries(test_opendog_algorithm mymit_opendog_algorithm)
    target_compile_definitions(test_opendog_algorithm PRIVATE
      MYMIT_OPENDOG_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/algorithm")
    ament_add_gtest(test_opendog_mujoco
      test/control/opendog_mujoco_test.cpp user/SimulationActuatorWriter.cpp)
    target_compile_definitions(test_opendog_mujoco PRIVATE
      MYMIT_ENABLE_OPENDOG
      MYMIT_OPENDOG_TEST_CONFIG="${CMAKE_CURRENT_SOURCE_DIR}/config/algorithm"
      MYMIT_ROBOT_TEST_SCENE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/dm1_description/dm1/scene.xml")
    target_include_directories(test_opendog_mujoco PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${CMAKE_CURRENT_SOURCE_DIR}/user)
    target_link_libraries(test_opendog_mujoco
      tinyxml2::tinyxml2 mymit_opendog_algorithm mymit_robot_model)
    target_link_options(test_opendog_mujoco PRIVATE -Wl,--no-as-needed)
    ament_target_dependencies(test_opendog_mujoco rclcpp)
    add_test(NAME test_opendog_model_contract
      COMMAND ${MYMIT_ROS_PYTHON_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/model/opendog_model_contract_test.py)
  endif()
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
    Middlewares/dm_imu/include
    Middlewares/dm_imu/src/bsp_crc.cpp
    Middlewares/dmbot_serial/include/dmbot_serial/protocol/socketcan.h
    Middlewares/dmbot_serial/include/dmbot_serial/protocol/terminal_colors.h
    Middlewares/dmbot_serial/src/protocol/socketcan.cpp
    include/common
    include/hardware
    include/model/robot_types.hpp
    include/teleop
    include/sensor
    src/controller/OrientationEstimator.cpp
    src/controller/PositionVelocityEstimator.cpp
    src/hardware
    src/sensor
    src/teleop
    test/teleop
    test/hardware
  )
  ament_xmllint(
    src/dm1_description/dm1/dm1.xml
    src/dm1_description/dm1/scene.xml
  )
endif()

ament_package()
