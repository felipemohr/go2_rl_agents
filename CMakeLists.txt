cmake_minimum_required(VERSION 3.8)
project(go2_rl_agents)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)

# Include cpp "include" directory
include_directories(include include/common)

# Create cpp executables
add_executable(go2_joint_controller_interface src/go2_joint_controller_interface.cpp src/common/motor_crc.cpp)
ament_target_dependencies(go2_joint_controller_interface rclcpp unitree_go unitree_api)

add_executable(go2_enable_motion_control src/go2_enable_motion_control.cpp)
ament_target_dependencies(go2_enable_motion_control rclcpp unitree_api)

# Install cpp executables
install(
  TARGETS go2_joint_controller_interface go2_enable_motion_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/go2_wireless_controller.py
  scripts/go2_agent_base.py
  scripts/go2_agent_cpg.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
