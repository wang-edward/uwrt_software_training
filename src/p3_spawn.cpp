#include "../include/software_training/p3_spawn.hpp"

namespace composition {

p3_spawn:: p3_spawn(const rclcpp::NodeOptions &options): Node{"p3_spawn", options} {

    client = this->create_client<turtlesim::srv::Spawn>("/spawn");

    for (auto turt : nodes) {

        auto message = std::make_unique<turtlesim::srv::Spawn::Request>();
        message->x = turt.x;
        message->y = turt.y;
        message->theta = turt.theta;
        message->name = turt.name;

        auto server_callback = [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) -> void {
            rclcpp::shutdown();
        };

        auto result = client->async_send_request(std::move(message), server_callback);
    }
} 
    

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p3_spawn)
