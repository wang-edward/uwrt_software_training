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
        void Spawn();
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client;
        rclcpp::TimerBase::SharedPtr timer;

        std::shared_ptr<turtlesim::srv::Spawn::Request> stationaryTurtle = std::make_shared<turtlesim::srv::Spawn::Request>();
        std::shared_ptr<turtlesim::srv::Spawn::Request> movingTurtle = std::make_shared<turtlesim::srv::Spawn::Request>();
        
};
} // namespace composition
