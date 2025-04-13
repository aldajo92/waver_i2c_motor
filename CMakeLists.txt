cmake_minimum_required(VERSION 3.8)
project(i2c_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Executable for motor_node
add_executable(motor_node src/motor_node.cpp)
ament_target_dependencies(motor_node rclcpp geometry_msgs)

# Link libi2c (available via libi2c-dev)
target_link_libraries(motor_node i2c)

# Executable for velocity_node
add_executable(velocity_node src/velocity_node.cpp)
ament_target_dependencies(velocity_node rclcpp geometry_msgs sensor_msgs)

# Link libi2c for velocity_node
target_link_libraries(velocity_node i2c)

# Install
install(TARGETS
  motor_node 
  velocity_node
  DESTINATION lib/${PROJECT_NAME}
)

# Optional Linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
