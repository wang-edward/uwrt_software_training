#pragma once

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

// ros2 libraries
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace composition {
    class circle_turtle_pub : public rclcpp::Node
    {
        public:
            circle_turtle_pub(const rclcpp::NodeOptions & options);
        
        private:
            //publisher to publish velocity msg
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
            // declaring a timer for the callback
            rclcpp::TimerBase::SharedPtr timer_;
            // veloctiy msg to be published to turtlesim
            geometry_msgs::msg::Twist velocity_msg;
            // coords for the msg
            struct 
            {
                struct 
                {
                    float x = 0.0;
                    float y = 0.0;
                    float z = 0.0;
                } linear;

                struct 
                {
                    float x = 0.0;
                    float y = 0.0;
                    float z = 2.0;
                } angular;
                
            } coordinates;
    };
}// namespace