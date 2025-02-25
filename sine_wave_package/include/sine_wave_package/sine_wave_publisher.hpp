#ifndef SINE_WAVE_PUBLISHER_HPP
#define SINE_WAVE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sine_wave_package/sine_wave_params.hpp>
#include <std_msgs/msg/float64.hpp>

class SineWavePublisher : public rclcpp::Node {
public:
    SineWavePublisher();
    void init();

private:
    void publish_sine_wave();

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sine_wave_package::Params params_;
};

#endif  // SINE_WAVE_PUBLISHER_HPP
