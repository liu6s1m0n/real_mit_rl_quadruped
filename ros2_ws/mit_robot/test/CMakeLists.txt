# Tests are grouped by subsystem so one layer can be built and debugged without
# pulling unrelated sources into the same executable.

function(add_robot_unit_test target source)
  ament_add_gtest(${target} ${source})
  target_link_libraries(${target} mymit_robot_model Eigen3::Eigen)
endfunction()

function(add_robot_mujoco_test target source)
  add_robot_unit_test(${target} ${source})
  target_compile_definitions(${target} PRIVATE
    MYMIT_ROBOT_TEST_SCENE_PATH="${PROJECT_SOURCE_DIR}/src/dm1_description/dm1/scene.xml"
  )
endfunction()

# Model and dynamics
add_robot_unit_test(test_model_dm1_load model/dm1_model_load_test.cpp)
target_compile_definitions(test_model_dm1_load PRIVATE
  MYMIT_ROBOT_DM1_TEST_SCENE_PATH="${PROJECT_SOURCE_DIR}/src/dm1_description/dm1/scene.xml"
)

# State and contact estimation
add_robot_unit_test(test_estimator_orientation estimation/orientation_estimator_test.cpp)

# Low-level leg, swing and gait control
add_robot_unit_test(test_control_swing control/foot_swing_trajectory_test.cpp)
add_robot_unit_test(test_control_gait control/gait_scheduler_test.cpp)

# MPC, WBC and FSM are split again by responsibility for focused debugging.
add_robot_unit_test(test_mpc_gait mpc/gait_test.cpp)
add_robot_unit_test(test_mpc_robot_state mpc/robot_state_test.cpp)
add_robot_unit_test(test_wbc_optimization wbc/whole_body_optimization_test.cpp)
add_robot_unit_test(test_fsm_safety fsm/safety_checker_test.cpp)
target_include_directories(test_fsm_safety PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/fsm
)
add_robot_unit_test(test_dm1_contract dm1_control_contract_test.cpp)
add_robot_unit_test(
  test_dm1_mit_interface hardware/dm1_mit_interface_test.cpp)

ament_add_gtest(
  test_dm1_sim2sim_direction
  sim2sim/dm1_direction_regression_test.cpp
  ${PROJECT_SOURCE_DIR}/user/RobotRunner.cpp
  ${PROJECT_SOURCE_DIR}/user/SimulationActuatorWriter.cpp
  ${PROJECT_SOURCE_DIR}/user/SimulationDiagnostics.cpp)
target_compile_definitions(test_dm1_sim2sim_direction PRIVATE
  MYMIT_ROBOT_TEST_SCENE_PATH="${PROJECT_SOURCE_DIR}/src/dm1_description/dm1/scene.xml"
)
target_include_directories(test_dm1_sim2sim_direction PRIVATE
  ${PROJECT_SOURCE_DIR}/user
)
target_link_libraries(test_dm1_sim2sim_direction mymit_robot_model Eigen3::Eigen)

ament_add_gtest(test_hardware_bridge
  hardware/hardware_bridge_test.cpp
  ${PROJECT_SOURCE_DIR}/user/HardwareBridge.cpp
  ${PROJECT_SOURCE_DIR}/user/RobotRunner.cpp
)
target_include_directories(test_hardware_bridge PRIVATE
  ${PROJECT_SOURCE_DIR}/user
)
target_link_libraries(test_hardware_bridge mymit_robot_model Eigen3::Eigen)
