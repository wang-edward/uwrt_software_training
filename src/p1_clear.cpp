#include "../include/software_training/p1_clear.hpp"

namespace composition {

p1_clear::p1_clear(const rclcpp::NodeOptions &options) 
    : Node("p1_clear", options){
    // complete this code

}

void p1_clear::kill() {
    // complete this code

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p1_clear)
