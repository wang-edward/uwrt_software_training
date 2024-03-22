#include "../include/uwrt_mars_rover/p2_circle_turtle.hpp"
#include <iostream>

using namespace std::chrono_literals;

namespace composition {

    circle_turtle_pub::circle_turtle_pub(const rclcpp::NodeOptions & options)
        : Node("circle_turtle_pub", options) 
        {
            //creating the publisher
            velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            
            
            auto publisher_callback = [this](void) -> void {
                auto velocity_msg = std::make_unique<geometry_msgs::msg::Twist>();
                velocity_msg->linear.x = this->coordinates.linear.x;
                velocity_msg->linear.y = this->coordinates.linear.y;
                velocity_msg->linear.z = this->coordinates.linear.z;

                velocity_msg->angular.x = this->coordinates.angular.x;
                velocity_msg->angular.y = this->coordinates.angular.y;
                velocity_msg->angular.z = this->coordinates.angular.z;

                this->velocity_publisher_->publish(std::move(velocity_msg));
            };

            timer_ = this->create_wall_timer(
                200ms, publisher_callback);
        }
} //namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::circle_turtle_pub)

