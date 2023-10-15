#include "../include/software_training/p2_circle.hpp"

using namespace std::chrono_literals;
namespace composition {

p2_circle::p2_circle(const rclcpp::NodeOptions& options) 
    : Node{"p2_circle", options} {
    // complete this code
    
}

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p2_circle)
