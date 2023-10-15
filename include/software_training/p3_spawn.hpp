#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <turtlesim/srv/spawn.hpp>

namespace composition {

class p3_spawn : public rclcpp::Node {
    public:
        p3_spawn(const rclcpp::NodeOptions &options);
    private:
        // complete this code
};

} // namespace composition
