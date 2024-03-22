#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

// ros2 libraries
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>

namespace composition {

class clear_node : public rclcpp::Node {
    
    public:
        //node
        clear_node(const rclcpp::NodeOptions & options);
    
    private:
        //client
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_;
        // timer
        rclcpp::TimerBase::SharedPtr timer_;
        // turtles (nodes) to kill (is there a way to kill all active turtlesim nodes?)
        std::vector<std::string> turtles = {"turtle1"};
        // function to kill turtle1
        void kill();
};


} //namespace