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

#include "sine_wave_subscriber.hpp"
#include <filesystem>

namespace fs = std::filesystem;

SineWaveSubscriber::SineWaveSubscriber()
: Node("sine_wave_subscriber"), start_time_(std::chrono::steady_clock::now())
{
  subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "/sine_wave", 10, std::bind(&SineWaveSubscriber::callback, this, std::placeholders::_1));

  std::string directory = "src/sine_wave_ros/sine_wave_logs";
  if (!fs::exists(directory)) {
    fs::create_directory(directory);
  }

    // open file
  std::string filepath = directory + "/sine_wave_data.csv";
  file_.open(filepath, std::ios::out | std::ios::trunc);

  if (!file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filepath.c_str());
    return;
  }

  if (fs::file_size(filepath) == 0) {
    file_ << "Time (s),Sine Value\n";
  }

  RCLCPP_INFO(this->get_logger(), "Sine wave subscriber started, saving to: %s", filepath.c_str());
}

void SineWaveSubscriber::callback(const std_msgs::msg::Float64::SharedPtr msg)
{
  auto now = std::chrono::steady_clock::now();
  double time_elapsed = std::chrono::duration<double>(now - start_time_).count();

  if (file_.is_open()) {
    file_ << time_elapsed << "," << msg->data << "\n";
    file_.flush();
  } else {
    RCLCPP_ERROR(this->get_logger(), "File is not open, data loss may occur!");
  }

  RCLCPP_INFO(this->get_logger(), "Received: %f at time %f", msg->data, time_elapsed);
}

#ifndef UNIT_TEST
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SineWaveSubscriber>();
  rclcpp::spin(node);
  return 0;
}
#endif
