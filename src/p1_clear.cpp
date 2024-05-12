#include "software_training/p1_clear.hpp"

namespace composition {

    p1_clear::p1_clear(const rclcpp::NodeOptions &options) 
        : Node("p1_clear", options){
        // complete this code
        kill();
    }

    void p1_clear::kill() {
        // complete this code
        client = this->create_client<turtlesim::srv::Kill>("/turtlesim1/kill");
        while(!(client->wait_for_service(std::chrono::seconds(1))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the kill server to go up");
        }

        auto killReq = std::make_shared<turtlesim::srv::Kill::Request>();
        killReq->name = "turtle1";

        auto callback2 =
            [this](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture response)
            -> void {
                (void)response;
            RCLCPP_INFO(this->get_logger(), "Turtle Killed");
            rclcpp::shutdown();
        };

        auto result = client->async_send_request(std::move(killReq), callback2);
    }
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p1_clear)

