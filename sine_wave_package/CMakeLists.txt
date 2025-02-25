cmake_minimum_required(VERSION 3.8)
project(sine_wave_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

include_directories(
  include/sine_wave_package
)

generate_parameter_library(
  sine_wave_params
  config/sine_wave_params.yaml
)

add_executable(sine_wave_publisher src/sine_wave_publisher.cpp)
add_executable(sine_wave_subscriber src/sine_wave_subscriber.cpp)

target_link_libraries(sine_wave_publisher rclcpp::rclcpp sine_wave_params)
target_link_libraries(sine_wave_subscriber rclcpp::rclcpp)

ament_target_dependencies(sine_wave_publisher rclcpp std_msgs)
ament_target_dependencies(sine_wave_subscriber rclcpp std_msgs)

install(TARGETS
  sine_wave_publisher
  sine_wave_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()