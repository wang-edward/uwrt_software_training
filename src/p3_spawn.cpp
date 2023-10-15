#include "../include/software_training/p3_spawn.hpp"

namespace composition {

p3_spawn:: p3_spawn(const rclcpp::NodeOptions &options): Node{"p3_spawn", options} {
    // complete this code
} 
    

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p3_spawn)
