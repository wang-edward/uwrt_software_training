#pragma once

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace composition {

class p2_circle : public rclcpp::Node {
    public:
        p2_circle(const rclcpp::NodeOptions& options);
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;

        static constexpr float linear_x = 1.2;
        static constexpr float linear_y = 1.2;
        static constexpr float linear_z = 1.2;
        static constexpr float angular_x = 1.41;
        static constexpr float angular_y = 1.41;
        static constexpr float angular_z = 1.41;

};


} // namespace composition