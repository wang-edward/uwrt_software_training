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
        void kill(); // function that kills all nodes
        // complete this code

};

} // namespace composition
