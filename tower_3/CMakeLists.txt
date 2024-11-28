cmake_minimum_required(VERSION 3.8)
project(tower_3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# If you have any executables, declare them here
# For example, if you have another source file for your main application
add_executable(tower_3 src/tower_3.cpp)

# Link against the required dependencies
ament_target_dependencies(tower_3 rclcpp gazebo_msgs moveit_visual_tools moveit_ros_planning_interface)

# Install targets (if you have any executables)
install(TARGETS tower_3
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright check
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint check
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#ament_package()
ament_auto_package(INSTALL_TO_SHARE launch)
