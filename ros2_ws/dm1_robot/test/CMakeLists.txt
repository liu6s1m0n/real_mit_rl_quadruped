# Tests are grouped by subsystem so one layer can be built and debugged without
# pulling unrelated sources into the same executable. OpenDog algorithm tests
# live in the top-level CMakeLists.txt next to the OpenDog targets.

ament_add_gtest(
  test_dm1_mit_interface hardware/dm1_mit_interface_test.cpp)
target_include_directories(test_dm1_mit_interface PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_dm1_mit_interface Eigen3::Eigen)

ament_add_gtest(test_teleop teleop/operator_command_test.cpp)
target_include_directories(test_teleop PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(test_teleop dm1_robot_teleop Eigen3::Eigen pthread)

ament_add_gtest(
  test_dm_motor_driver
  hardware/dm_motor_driver_test.cpp
  ${PROJECT_SOURCE_DIR}/src/hardware/dm_motor_driver.cpp)
target_include_directories(test_dm_motor_driver PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/third_party/dmbot_serial/include
)
target_link_libraries(test_dm_motor_driver
  dm1_dmbot_serial Eigen3::Eigen pthread)

ament_add_gtest(
  test_imu_driver
  hardware/imu_driver_test.cpp
  ${PROJECT_SOURCE_DIR}/src/sensor/imu_driver.cpp
  ${PROJECT_SOURCE_DIR}/third_party/dm_imu/src/bsp_crc.cpp)
target_include_directories(test_imu_driver PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/third_party/dm_imu/include
)
target_link_libraries(test_imu_driver pthread util)

ament_add_gtest(test_dm1_full_hardware_simulation
  simulation/dm1_full_hardware_simulation_test.cpp
  ${PROJECT_SOURCE_DIR}/src/simulation/HardwareSimulation.cpp TIMEOUT 90)
target_compile_definitions(test_dm1_full_hardware_simulation PRIVATE
  DM1_ROBOT_TEST_SCENE_PATH="${PROJECT_SOURCE_DIR}/description/scene.xml"
  DM1_ROBOT_TEST_CALIBRATION_PATH="${PROJECT_SOURCE_DIR}/config/hardware/dm1_hardware_calibration.txt"
  DM1_OPENDOG_TEST_CONFIG="${PROJECT_SOURCE_DIR}/config/opendog")
target_include_directories(test_dm1_full_hardware_simulation PRIVATE
  ${PROJECT_SOURCE_DIR}/third_party/dm_imu/include)
# This test intentionally exercises the ROS/OpenDog dependency order.
target_link_libraries(test_dm1_full_hardware_simulation
  tinyxml2::tinyxml2 dm1_robot_hardware util)
target_link_options(test_dm1_full_hardware_simulation PRIVATE -Wl,--no-as-needed)

ament_add_gtest(test_hardware_bridge
  hardware/hardware_bridge_test.cpp
)
target_include_directories(test_hardware_bridge PRIVATE
  ${PROJECT_SOURCE_DIR}/include
)
target_compile_definitions(test_hardware_bridge PRIVATE
  DM1_OPENDOG_TEST_CONFIG="${PROJECT_SOURCE_DIR}/config/opendog")
target_link_libraries(test_hardware_bridge tinyxml2::tinyxml2 dm1_robot_hardware)
target_link_options(test_hardware_bridge PRIVATE -Wl,--no-as-needed)
