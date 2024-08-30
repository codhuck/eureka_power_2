#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <array>
#include <vector>
#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string> 
#include <cstring>

namespace battery_decoder 
{
    class Battery_decoder: public rclcpp::Node
    {
        public:

            Battery_decoder();
            ~Battery_decoder();

private:

    void callback(const std_msgs::msg::UInt8MultiArray::SharedPtr arr);
    void publisher();
    float convertFloat16ToFloat32(uint8_t byte1, uint8_t byte2);

private:

    std::array<float, 3> voltage;
    std::array<float, 3> current;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub1;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub2;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub3;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub;
    rclcpp::TimerBase::SharedPtr timer;
    };

}