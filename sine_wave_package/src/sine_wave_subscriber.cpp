#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class SineWaveSubscriber : public rclcpp::Node
{
public:
    SineWaveSubscriber()
        : Node("sine_wave_subscriber")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/sine_wave", 10, std::bind(&SineWaveSubscriber::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Sine wave subscriber started");
    }

private:
    void callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: %f", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SineWaveSubscriber>());
    rclcpp::shutdown();
    return 0;
}
