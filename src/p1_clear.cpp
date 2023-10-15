#include "../include/software_training/p1_clear.hpp"

namespace composition {

p1_clear::p1_clear(const rclcpp::NodeOptions &options) : Node("p1_clear", options){
    // why pass options?
    client = this->create_client<turtlesim::srv::Kill>("kill");
    // syntax: create_client<service type>(name)
    timer_ = create_wall_timer(
        std::chrono::duration
            <int, std::chrono::seconds::period>(1), 
        std::bind(&p1_clear::kill, this)
        // bind "this" to the first (only param) of kill()
    );
}

void p1_clear::kill() {

    for (auto turtle : nodes) {
        auto req = std::make_shared<turtlesim::srv::Kill::Request>();
        req->name = turtle; // otherwise tries to kill "turtle []"
        auto callback = [this, turtle](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture response) -> void {
            std::cout<<"killed: "<<turtle<<std::endl;
            auto output = response.get();
            std::cout<<"response: "<<output<<std::endl;
            rclcpp::shutdown();
        };
        
        auto result = client->async_send_request(req, callback);
    }

}

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p1_clear)
