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

#include "grayscale_server.hpp"
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

GrayscaleServer::GrayscaleServer()
: Node("grayscale_server")
{
  service_ = create_service<sine_wave_package::srv::ConvertToGrayscale>(
            "convert_to_grayscale",
            std::bind(&GrayscaleServer::handle_service, this, _1, _2));
  RCLCPP_INFO(this->get_logger(), "Grayscale conversion server started, waiting for requests...");
}

void GrayscaleServer::handle_service(
  const std::shared_ptr<sine_wave_package::srv::ConvertToGrayscale::Request> request,
  std::shared_ptr<sine_wave_package::srv::ConvertToGrayscale::Response> response)
{
    // Read the color image
  cv::Mat color_image = cv::imread(request->color_image_path, cv::IMREAD_COLOR);
  if (color_image.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Unable to read image: %s",
            request->color_image_path.c_str());
    return;
  }

    // Convert to grayscale
  cv::Mat gray_image;
  // cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
  try {
    cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);
  } catch (const cv::Exception &e) {
    RCLCPP_ERROR(this->get_logger(),
      "OpenCV exception during color conversion: %s", e.what());
    return;
  }

    // Fill the sensor_msgs/Image message
  response->grayscale_image.height = gray_image.rows;
  response->grayscale_image.width = gray_image.cols;
  response->grayscale_image.encoding = "mono8";   // Single channel encoding for grayscale
  response->grayscale_image.step = gray_image.cols;   // Bytes per row
  response->grayscale_image.data.assign(gray_image.data,
        gray_image.data + gray_image.total());

  RCLCPP_INFO(this->get_logger(), "Processed image: %s, size %dx%d",
                            request->color_image_path.c_str(),
                            gray_image.cols, gray_image.rows);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try{
    auto node = std::make_shared<GrayscaleServer>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), e.what());
  }
  rclcpp::shutdown();
  return 0;
}
