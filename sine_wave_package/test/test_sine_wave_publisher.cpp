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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sine_wave_package/sine_wave_params.hpp>
#include "sine_wave_publisher.hpp"

class SineWavePublisherTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    node_ = std::make_shared<SineWavePublisher>();
    node_->init();
  }

  void TearDown() override
  {
    // rclcpp::shutdown();  // 移到 main() 统一关闭
  }

  std::shared_ptr<SineWavePublisher> node_;
};

TEST_F(SineWavePublisherTest, Initialization) {
    EXPECT_STREQ(node_->get_name(), "sine_wave_publisher");
}

TEST_F(SineWavePublisherTest, MessagePublishing) {
    auto sub_node = std::make_shared<rclcpp::Node>("test_subscriber");
    auto received = false;
    auto message_callback = [&](std_msgs::msg::Float64::SharedPtr msg) {
      received = true;
      EXPECT_LE(std::abs(msg->data), 10);
    };

    auto subscriber = sub_node->create_subscription<std_msgs::msg::Float64>(
        "/sine_wave", 10, message_callback);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_);
    executor.add_node(sub_node);

    // 增加等待时间，确保 subscriber 有机会接收消息
    for (int i = 0; i < 10; ++i) {
    executor.spin_some();
    if (received) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    EXPECT_TRUE(received);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);    // 统一在 main() 里初始化 ROS
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();    // 确保测试结束后关闭 ROS
  return result;
}
