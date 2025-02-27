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

  if (params_.publisher.frequency <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Frequency must be greater than 0");
    return;
  }

  if (std::isinf(params_.publisher.amplitude) || std::isnan(params_.publisher.amplitude)) {
    RCLCPP_ERROR(this->get_logger(),
      "Invalid amplitude: %f. Must be a finite number.", params_.publisher.amplitude);
    throw std::invalid_argument("Amplitude must be finite");
  }

  if (params_.publisher.angular_frequency < 0.0) {
    RCLCPP_ERROR(this->get_logger(),
      "Invalid angular frequency: %f. Must be >= 0.", params_.publisher.angular_frequency);
    throw std::invalid_argument("Angular frequency must be >= 0");
  }

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

double SineWavePublisher::get_amplitude()
{
  return params_.publisher.amplitude;
}

#ifndef UNIT_TEST
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<SineWavePublisher>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
  }
  catch(const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), e.what());
    rclcpp::shutdown();
    return 1;
  }
  return 0;
}
#endif
