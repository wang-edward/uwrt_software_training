#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <turtlesim/srv/kill.hpp>

namespace composition {
class p1_clear : public rclcpp::Node {
    public:
        p1_clear(const rclcpp::NodeOptions &options);
    private:
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;
        std::vector<std::string> nodes = {"turtle1"}; 
        rclcpp::TimerBase::SharedPtr timer_;
        void kill();

};

} // namespace composition
