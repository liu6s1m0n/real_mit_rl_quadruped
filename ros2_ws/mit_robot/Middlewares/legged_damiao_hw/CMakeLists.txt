cmake_minimum_required(VERSION 3.10)
project(legged_damiao_hw)
#<!--launch-prefix="gdb -ex run --args"/>
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
        dmbot_serial
)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(pinocchio REQUIRED pinocchio)


catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        legged_hw
        dmbot_serial
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
       # ${pinocchio_INCLUDE_DIRS}
)

# link_directories(
#         ${pinocchio_LIBRARY_DIRS}
# )

link_directories(lib)

set(SOURCE src/legged_dm_hw.cpp src/DmHW.cpp)

add_executable(${PROJECT_NAME}
        ${SOURCE}
        )

target_compile_definitions(${PROJECT_NAME} PRIVATE )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}

       # ${pinocchio_LIBRARIES}
        )



#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} TARGETS 
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_damiao_hw
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/legged_damiao_hw
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/ 
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
