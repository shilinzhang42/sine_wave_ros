cmake_minimum_required(VERSION 3.8)
project(sine_wave_package)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include/sine_wave_package)

generate_parameter_library(sine_wave_params config/sine_wave_params.yaml)

add_executable(sine_wave_publisher src/sine_wave_publisher.cpp)
add_executable(sine_wave_subscriber src/sine_wave_subscriber.cpp)

target_link_libraries(sine_wave_publisher rclcpp::rclcpp sine_wave_params)
target_link_libraries(sine_wave_subscriber rclcpp::rclcpp)

ament_target_dependencies(sine_wave_publisher rclcpp std_msgs)
ament_target_dependencies(sine_wave_subscriber rclcpp std_msgs)

install(TARGETS sine_wave_publisher sine_wave_subscriber
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

if(BUILD_TESTING)
  add_library(sine_wave_publisher_lib_test STATIC src/sine_wave_publisher.cpp)
  add_library(sine_wave_subscriber_lib_test STATIC src/sine_wave_subscriber.cpp)
  target_compile_definitions(sine_wave_publisher_lib_test PRIVATE "UNIT_TEST")
  target_compile_definitions(sine_wave_subscriber_lib_test PRIVATE "UNIT_TEST")
  target_link_libraries(sine_wave_publisher_lib_test rclcpp::rclcpp
                        sine_wave_params)
  target_link_libraries(sine_wave_subscriber_lib_test rclcpp::rclcpp)
  ament_target_dependencies(sine_wave_publisher_lib_test rclcpp std_msgs)
  ament_target_dependencies(sine_wave_subscriber_lib_test rclcpp std_msgs)

  ament_add_gtest(test_sine_wave_publisher test/test_sine_wave_publisher.cpp)
  ament_add_gtest(test_sine_wave_subscriber test/test_sine_wave_subscriber.cpp)
  if(TARGET test_sine_wave_publisher)
    target_link_libraries(test_sine_wave_publisher sine_wave_publisher_lib_test)
  endif()
  if(TARGET test_sine_wave_subscriber)
    target_link_libraries(test_sine_wave_subscriber
                          sine_wave_subscriber_lib_test)
  endif()
endif()

rosidl_generate_interfaces(${PROJECT_NAME} "srv/ConvertToGrayscale.srv"
                           DEPENDENCIES sensor_msgs)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

add_executable(server src/grayscale_server.cpp)
ament_target_dependencies(server rclcpp sensor_msgs OpenCV)
rosidl_target_interfaces(server ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(client src/grayscale_client.cpp)
ament_target_dependencies(client rclcpp sensor_msgs OpenCV)
rosidl_target_interfaces(client ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS server client DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

ament_package()
