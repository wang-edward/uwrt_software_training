#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

// ros2 libraries
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

namespace composition {
class spawn_node : public rclcpp::Node {
    
    public:
        //node
        spawn_node(const rclcpp::NodeOptions & options);
    
    private:
        //client
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
        // coords for the srv
        struct TurtleInfo {
            float x_pos;
            float y_pos;
            float theta;
        };
        // turtles (nodes) to spawn
        std::vector<std::string> turtles = {"stationary_turtle", "moving_turtle", "turtle1"};
        // locations to spawn
        std::vector<TurtleInfo> turtle_loc{
            {5, 5, 0}, // coords for stationary turtle
            {25, 10, 0}, // coords for moving turtle
            {5, 8, 0} // coords for turtle1
        };
        // map of turtle name to turtle information
        std::map<std::string, TurtleInfo> turtle_description;
        // function to kill turtle1
        void spawn_turts();
};
} // namespace