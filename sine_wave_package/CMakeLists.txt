cmake_minimum_required(VERSION 3.8)
project(sine_wave_package)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

# 生成参数库
generate_parameter_library(
  sine_wave_params
  config/sine_wave_params.yaml
)

# 先定义可执行目标
add_executable(sine_wave_publisher src/sine_wave_publisher.cpp)
add_executable(sine_wave_subscriber src/sine_wave_subscriber.cpp)

# 统一使用 `PRIVATE` 关键字
target_link_libraries(sine_wave_publisher rclcpp::rclcpp sine_wave_params)
target_link_libraries(sine_wave_subscriber rclcpp::rclcpp)

# 统一 `ament_target_dependencies`
ament_target_dependencies(sine_wave_publisher rclcpp std_msgs)
ament_target_dependencies(sine_wave_subscriber rclcpp std_msgs)

# 安装可执行文件
install(TARGETS
  sine_wave_publisher
  sine_wave_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 Launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
