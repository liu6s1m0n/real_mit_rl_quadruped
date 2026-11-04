cmake_minimum_required(VERSION 3.5)
project(dmbot_serial)

# ========== 1. 检测 ROS 版本 ==========
if($ENV{ROS_VERSION} EQUAL 1)
  set(USE_ROS1 TRUE)
  message(STATUS ">>> Building for ROS 1 (Noetic)")
elseif($ENV{ROS_VERSION} EQUAL 2)
  set(USE_ROS2 TRUE)
  message(STATUS ">>> Building for ROS 2 (Foxy/Humble)")
else()
  message(STATUS ">>> Building in pure-CMake mode")
endif()

# ========== 2. 通用编译 flags ==========
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)

# ========== 3. ROS 1 分支 ==========
if(USE_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES motor
    CATKIN_DEPENDS roscpp
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  # 生成库
  add_library(motor
    src/protocol/damiao.cpp
    src/protocol/socketcan.cpp
  )
  target_link_libraries(motor ${catkin_LIBRARIES} Threads::Threads)

  # 生成可执行文件
  add_executable(test_motor src/test.cpp)
  target_link_libraries(test_motor motor ${catkin_LIBRARIES})

  install(TARGETS motor test_motor
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# ========== 4. ROS 2 分支 ==========
elseif(USE_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  include_directories(include)

  # 生成库
  add_library(motor
    src/protocol/damiao.cpp
    src/protocol/socketcan.cpp
  )
  ament_target_dependencies(motor rclcpp std_msgs)
  target_link_libraries(motor Threads::Threads)

  # 生成可执行文件
  add_executable(test_motor src/test.cpp)
  ament_target_dependencies(test_motor rclcpp std_msgs)
  target_link_libraries(test_motor motor)

  install(TARGETS motor test_motor
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/ DESTINATION include)

  ament_package()

# ========== 5. 纯 CMake 分支 ==========
else()
  include_directories(include)

  add_library(motor
    src/protocol/damiao.cpp
    src/protocol/socketcan.cpp
  )
  target_link_libraries(motor Threads::Threads)

  add_executable(test_motor src/test.cpp)
  target_link_libraries(test_motor motor)

  install(TARGETS motor test_motor
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
  install(DIRECTORY include/ DESTINATION include)
endif()