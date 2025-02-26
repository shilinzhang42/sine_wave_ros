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
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "sine_wave_subscriber.hpp"


namespace fs = std::filesystem;

// Define a simple test publisher node to publish /sine_wave messages
class TestPublisher : public rclcpp::Node {
public:
  TestPublisher()
  : Node("test_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sine_wave", 10);
  }

  void publish_message(double value)
  {
    auto msg = std::make_shared<std_msgs::msg::Float64>();
    msg->data = value;
    publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Published test message: %f", value);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

class SineWavePublisherTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
        // Clean up log files generated during the test
    std::string directory = "/home/ws/src/sine_wave_ros/sine_wave_logs";
    std::string filepath = directory + "/sine_wave_data.csv";
    if (fs::exists(filepath)) {
      fs::remove(filepath);
    }
  }
};

TEST_F(SineWavePublisherTest, test_publisher) {
    // Create a single-threaded executor
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    // Instantiate the SineWaveSubscriber node
    // (which will automatically create the file and write the header)
    auto subscriber_node = std::make_shared<SineWaveSubscriber>();
    // Instantiate the test publisher node
    auto test_publisher = std::make_shared<TestPublisher>();

    // Add nodes to the executor
    executor->add_node(subscriber_node);
    executor->add_node(test_publisher);

    // Publish a test message
    double test_value = 2.345;
    test_publisher->publish_message(test_value);

    // Allow some time to process the message and write to the file
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
    executor->spin_once(std::chrono::milliseconds(100));
    }

    // Check if the log file exists
    std::string directory = "/home/ws/src/sine_wave_ros/sine_wave_logs";
    std::string filepath = directory + "/sine_wave_data.csv";
    EXPECT_TRUE(fs::exists(filepath));

    // Read the file content and verify
    std::ifstream file(filepath);
    ASSERT_TRUE(file.is_open());

    std::string header;
    std::getline(file, header);
    EXPECT_EQ(header, "Time (s),Sine Value");

    std::string data_line;
    std::getline(file, data_line);
    // Ensure the data line is not empty
    EXPECT_FALSE(data_line.empty());

    // Parse the data line to extract the sine value field
    size_t comma_pos = data_line.find(',');
    ASSERT_NE(comma_pos, std::string::npos);
    std::string value_str = data_line.substr(comma_pos + 1);
    double sine_value = std::stod(value_str);
    EXPECT_NEAR(sine_value, test_value, 0.001);

    file.close();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
