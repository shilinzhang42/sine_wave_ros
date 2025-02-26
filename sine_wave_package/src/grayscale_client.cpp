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

#include "rclcpp/rclcpp.hpp"
#include "sine_wave_package/srv/convert_to_grayscale.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

class GrayscaleClient : public rclcpp::Node {
public:
  GrayscaleClient()
  : Node("grayscale_client")
  {
    client_ = create_client<sine_wave_package::srv::ConvertToGrayscale>("convert_to_grayscale");
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Client failed to start");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service to come online...");
    }
  }

  void send_request()
  {
    auto request = std::make_shared<sine_wave_package::srv::ConvertToGrayscale::Request>();
    request->color_image_path = "/home/ws/src/sine_wave_ros/plot/sine_wave_plot.png";

    auto result = client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Request sent: %s", request->color_image_path.c_str());

    if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = result.get();
      RCLCPP_INFO(this->get_logger(), "Received grayscale image, size: %dx%d",
                        response->grayscale_image.width, response->grayscale_image.height);
      cv::Mat gray_image(response->grayscale_image.height, response->grayscale_image.width,
        CV_8UC1, response->grayscale_image.data.data());
      cv::imwrite("/home/ws/src/sine_wave_ros/plot/output_gray.jpg", gray_image);
      RCLCPP_INFO(this->get_logger(), "Grayscale image saved to output_gray.jpg");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Request failed");
    }
  }

  rclcpp::Client<sine_wave_package::srv::ConvertToGrayscale>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GrayscaleClient>();
  node->send_request();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
