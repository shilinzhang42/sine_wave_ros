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

  // create directory if it doesn't exist
  try {
    if (!fs::exists(directory)) {
      if (!fs::create_directory(directory)) {
        RCLCPP_ERROR(this->get_logger(),
          "Failed to create directory: %s (unknown reason)", directory.c_str());
        throw std::runtime_error("Directory creation failed");
      }
    }
  } catch (const fs::filesystem_error &e) {
    RCLCPP_ERROR(this->get_logger(), "Filesystem error: %s", e.what());
    throw;  
  }

  // open file
  std::string filepath = directory + "/sine_wave_data.csv";
  file_.open(filepath, std::ios::out | std::ios::trunc);

  // check if file is open
  if (!file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filepath.c_str());
    return;
  }

  // write header if file is empty
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
  try{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SineWaveSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), e.what());
    return 1;
  }
  
  return 0;
}
#endif
