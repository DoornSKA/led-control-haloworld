cmake_minimum_required(VERSION 3.8)
project(serial_battery_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(control_msg REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(publisher src/serial_reader.cpp)
ament_target_dependencies(publisher rclcpp control_msg)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
