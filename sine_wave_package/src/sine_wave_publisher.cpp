#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sine_wave_package/sine_wave_params.hpp>  // 生成的参数库
#include <cmath>

class SineWavePublisher : public rclcpp::Node
{
public:
    SineWavePublisher()
        : Node("sine_wave_publisher")
    {
        
    }

    void init(){
        auto param_listener = std::make_shared<sine_wave_package::ParamListener>
            (this->shared_from_this());
        params_ = param_listener->get_params();

        // 创建发布者
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sine_wave", 10);

        // 定时发布
        double period = 1.0 / params_.publisher.frequency;
        timer_ = this->create_wall_timer(std::chrono::duration<double>(period),
                                         std::bind(&SineWavePublisher::publish_sine_wave, this));

        RCLCPP_INFO(this->get_logger(), "Sine wave publisher started at %f Hz", params_.publisher.frequency);
    }

private:
    void publish_sine_wave()
    {
        double time = this->now().seconds();
        double sine_value = params_.publisher.amplitude * std::sin(params_.publisher.angular_frequency * time + params_.publisher.phase);

        auto message = std_msgs::msg::Float64();
        message.data = sine_value;
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published: %f", sine_value);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sine_wave_package::Params params_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::shared_ptr<SineWavePublisher>(new SineWavePublisher());
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
