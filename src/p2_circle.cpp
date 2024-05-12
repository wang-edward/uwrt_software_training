#include "software_training/p2_circle.hpp"

using namespace std::chrono_literals;
namespace composition {

    p2_circle::p2_circle(const rclcpp::NodeOptions& options): Node{"p2_circle", options} {

        auto callback = [this](void) -> void {
            auto velocity = geometry_msgs::msg::Twist();
            velocity.linear.x = 3;
            velocity.angular.z = -1;
            this->publisher->publish(velocity);
        };

        
        this->publisher= this->create_publisher<geometry_msgs::msg::Twist>("/turtlesim1/moving_turtle/cmd_vel", 10);
        this->timer = this->create_wall_timer(std::chrono::milliseconds(500), callback);
        RCLCPP_INFO(this->get_logger(), "velocity publisher has been started.");
    }

    

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p2_circle)