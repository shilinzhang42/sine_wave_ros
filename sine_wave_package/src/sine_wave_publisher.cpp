// MIT License
//
// Copyright (c) 2025 Shilin Zhang
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "sine_wave_publisher.hpp"
SineWavePublisher::SineWavePublisher()
: Node("sine_wave_publisher") {}

void SineWavePublisher::init()
{
  auto param_listener =
    std::make_shared<sine_wave_package::ParamListener>(this->shared_from_this());
  params_ = param_listener->get_params();
  publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sine_wave", 10);

  double period = 1.0 / params_.publisher.frequency;
  timer_ = this->create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&SineWavePublisher::publish_sine_wave, this));

  RCLCPP_INFO(this->get_logger(), "Sine wave publisher started at %f Hz",
    params_.publisher.frequency);
}

void SineWavePublisher::publish_sine_wave()
{
  double time = this->now().seconds();
  double sine_value = params_.publisher.amplitude *
    std::sin(params_.publisher.angular_frequency * time + params_.publisher.phase);

  auto message = std_msgs::msg::Float64();
  message.data = sine_value;
  publisher_->publish(message);

  RCLCPP_INFO(this->get_logger(), "Published: %f", sine_value);
}

#ifndef UNIT_TEST
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SineWavePublisher>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
