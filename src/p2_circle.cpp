#include "../include/software_training/p2_circle.hpp"
#include <iostream>

using namespace std::chrono_literals;
namespace composition {

p2_circle::p2_circle(const rclcpp::NodeOptions& options) 
    : Node{"p2_circle", options} {
    
    std::cout<<"\nSTARTED"<<std::endl;
    auto publish_callback = [this](void) -> void {

        auto message = std::make_unique<geometry_msgs::msg::Twist>();
        message->linear.x = p2_circle::linear_x;
        message->linear.y = this->linear_y;
        message->linear.z = this->linear_z;

        message->angular.x = this->angular_x;
        message->angular.y = this->angular_y;
        message->angular.z = this->angular_z;

        // send velocity message with move constructor
        publisher->publish(std::move(message));
    };

    this->publisher = this->create_publisher<geometry_msgs::msg::Twist>
        ("turtle1/cmd_vel", 10);
    this->timer = this->create_wall_timer(100ms, publish_callback);

}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p2_circle)
