################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(bear_actuator_dfm_ros)

# add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED 
  COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    controller_manager
    hardware_interface
    # westwood_legged_control
    westwood_legged_msgs
)

## YAML-CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp
  controller_manager
  hardware_interface
  # westwood_legged_control
  westwood_legged_msgs
)

set(SOURCES src/cbear/port_manager.cpp src/cbear/packet_manager.cpp src/cbear/bear_sdk.cpp)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_library(cbear OBJECT ${SOURCES})
add_library(gim OBJECT src/gim/gim.cpp)


add_executable(bear_control_node src/bear_control_node.cpp src/hardware_interface.cpp)
add_dependencies(bear_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bear_control_node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} cbear gim)

add_executable(sample_main src/app/sample_main/sample_main.cpp)
target_link_libraries(sample_main ${catkin_LIBRARIES} cbear)

add_executable(gim_calibration src/gim/gim_calibration.cpp)
target_link_libraries(gim_calibration ${catkin_LIBRARIES} gim)

################################################################################
# Install
################################################################################
install(TARGETS bear_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
