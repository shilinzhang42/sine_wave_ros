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

#ifndef SINE_WAVE_PACKAGE__GRAYSCALE_SERVER_HPP_
#define SINE_WAVE_PACKAGE__GRAYSCALE_SERVER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sine_wave_package/srv/convert_to_grayscale.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

class GrayscaleServer : public rclcpp::Node {
public:
  GrayscaleServer();

private:
  void handle_service(
    const std::shared_ptr<sine_wave_package::srv::ConvertToGrayscale::Request> request,
    std::shared_ptr<sine_wave_package::srv::ConvertToGrayscale::Response> response);

  rclcpp::Service<sine_wave_package::srv::ConvertToGrayscale>::SharedPtr service_;
};

#endif  //  SINE_WAVE_PACKAGE__GRAYSCALE_SERVER_HPP_
