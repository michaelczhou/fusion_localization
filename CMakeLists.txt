cmake_minimum_required(VERSION 3.5)
project(fusion_localization LANGUAGES C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(EXTERNAL_PROJECT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_program(CCACHE ccache)
if(CCACHE)
    message(STATUS "Found ccache: " ${CCACHE})
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ${CCACHE})
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ${CCACHE})
endif(CCACHE)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp rosbag rospy
    std_msgs geometry_msgs nav_msgs sensor_msgs
    tf cv_bridge image_transport)# camera_models)

#include(DownloadProject)
#download_project(
#    PROJ                loguru
#    GIT_REPOSITORY      https://github.com/emilk/loguru.git
#    GIT_TAG             v2.0.0
#    UPDATE_DISCONNECTED 1)
#
#file(COPY ${loguru_SOURCE_DIR}/loguru.hpp DESTINATION ${EXTERNAL_PROJECT_DIR})
#file(COPY ${loguru_SOURCE_DIR}/loguru.cpp DESTINATION ${EXTERNAL_PROJECT_DIR})

catkin_package()

add_library(external_lib SHARED
    ${EXTERNAL_PROJECT_DIR}/loguru.cpp
    ${EXTERNAL_PROJECT_DIR}/loguru.hpp)
target_compile_definitions(external_lib PUBLIC
    LOGURU_DEBUG_CHECKS=0
    LOGURU_DEBUG_LOGGING=0
    LOGURU_STACKTRACES=0
    LOGURU_UNSAFE_SIGNAL_HANDLER=0
    LOGURU_WITH_STREAMS=1)
target_include_directories(external_lib INTERFACE
    SYSTEM ${Eigen3_INCLUDE_DIRS}
           ${catkin_INCLUDE_DIRS}
           ${Ceres_INCLUDE_DIRS}
           ${PCL_INCLUDE_DIRS}
           ${OpenCV_INCLUDE_DIRS}
           ${EXTERNAL_PROJECT_DIR})
target_link_libraries(external_lib INTERFACE
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES})

message(STATUS "Ceres: ${CERES_LIBRARIES}")

add_library(fusion_lib SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/lidar_factor.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/math.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/estimator.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/state_parameterization.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/wheel_factor.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/wheel_integrator.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/estimation/wheel_integrator.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/kitti_adapter.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/kitti_adapter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/utility.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/utility.cpp)

target_link_libraries(fusion_lib PRIVATE external_lib)
target_compile_options(fusion_lib PRIVATE -Wall -pedantic -fdiagnostics-color=always -O3)
set_target_properties(fusion_lib PROPERTIES
        CXX_EXTENSIONS OFF
        LINKER_LANGUAGE CXX)

add_executable(laser_regist ${CMAKE_CURRENT_SOURCE_DIR}/src/node/lidar_registration.cc)
target_compile_options(laser_regist PRIVATE -Wall -pedantic -fdiagnostics-color=always -O3)
target_link_libraries(laser_regist PRIVATE fusion_lib external_lib)

add_executable(laser_odo ${CMAKE_CURRENT_SOURCE_DIR}/src/node/lidar_odometry.cc)
target_compile_options(laser_odo PRIVATE -Wall -pedantic -fdiagnostics-color=always -O3)
target_link_libraries(laser_odo PRIVATE fusion_lib external_lib)

add_executable(laser_map ${CMAKE_CURRENT_SOURCE_DIR}/src/node/lidar_mapping.cc)
target_compile_options(laser_map PRIVATE -Wall -pedantic -fdiagnostics-color=always -O3)
target_link_libraries(laser_map PRIVATE fusion_lib external_lib)

add_executable(kittiAdapter ${CMAKE_CURRENT_SOURCE_DIR}/src/node/kitti_adapter.cc)
target_compile_options(kittiAdapter PRIVATE -Wall -pedantic -fdiagnostics-color=always -O3)
target_link_libraries(kittiAdapter PRIVATE fusion_lib external_lib)
