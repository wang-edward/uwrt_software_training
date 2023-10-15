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
        // complete this code
};


} // namespace composition
