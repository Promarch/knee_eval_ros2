cmake_minimum_required(VERSION 3.5)
project(bota_ft_sensor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(include)

# Create the Bota FT sensor communication library
add_library(bota_ft_sensor_comm
  src/BotaForceTorqueSensorComm.cpp
)

# Create the ROS2 node executable
add_executable(bota_ft_sensor_node src/bota_ft_sensor_ros2_node.cpp)
target_link_libraries(bota_ft_sensor_node
  bota_ft_sensor_comm
)
ament_target_dependencies(bota_ft_sensor_node
  rclcpp
  geometry_msgs
  std_srvs
)

# Install the node executable
install(TARGETS
  bota_ft_sensor_node
  bota_ft_sensor_comm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
