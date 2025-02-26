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

#ifndef SINE_WAVE_PACKAGE__SINE_WAVE_PUBLISHER_HPP_
#define SINE_WAVE_PACKAGE__SINE_WAVE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sine_wave_package/sine_wave_params.hpp>
#include <std_msgs/msg/float64.hpp>

class SineWavePublisher : public rclcpp::Node {
public:
  SineWavePublisher();
  void init();
  double get_amplitude();

private:
  void publish_sine_wave();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  sine_wave_package::Params params_;
};

#endif  // SINE_WAVE_PACKAGE__SINE_WAVE_PUBLISHER_HPP_
