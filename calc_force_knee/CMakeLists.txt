cmake_minimum_required(VERSION 3.8)
project(calc_force_knee)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
include_directories("include/")

add_executable(CalcForceKnee src/calc_force_knee.cpp)
ament_target_dependencies(CalcForceKnee
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_msgs
)

add_executable(ForceVisualizer src/visualize_force.cpp)
ament_target_dependencies(ForceVisualizer
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_msgs
  visualization_msgs
)

INSTALL(TARGETS
  CalcForceKnee
  ForceVisualizer
  DESTINATION lib/${PROJECT_NAME}
)

INSTALL(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
