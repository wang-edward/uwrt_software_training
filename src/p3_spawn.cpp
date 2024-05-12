#include "software_training/p3_spawn.hpp"

namespace composition {

    p3_spawn:: p3_spawn(const rclcpp::NodeOptions &options)
        : Node{"p3_spawn", options} {
        // complete this code

        stationaryTurtle->x = 5;
        stationaryTurtle->y = 5;
        stationaryTurtle->name = "stationary_turtle";

        movingTurtle->x = 5;
        movingTurtle->y = 10;
        movingTurtle->name = "moving_turtle";

        Spawn();
    } 
    
    void p3_spawn::Spawn() {
        // complete this code
        client = this->create_client<turtlesim::srv::Spawn>("/turtlesim1/spawn");
        while(!(client->wait_for_service(std::chrono::seconds(1))))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the spawn server to go up");
        }

        auto SpawnRequest = std::make_shared<turtlesim::srv::Spawn::Request>();
        SpawnRequest->x = stationaryTurtle->x;
        SpawnRequest->y = stationaryTurtle->y;
        SpawnRequest->name = stationaryTurtle->name;

        auto callback =
            [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response)
            -> void {
        RCLCPP_INFO(this->get_logger(), "Turtle Created: %s",
                    response.get()->name.c_str());
        };

        auto result = client->async_send_request(std::move(SpawnRequest), callback);

        auto SpawnRequest2 = std::make_shared<turtlesim::srv::Spawn::Request>();
        SpawnRequest2->x = movingTurtle->x;
        SpawnRequest2->y = movingTurtle->y;
        SpawnRequest2->name = movingTurtle->name;

        auto callback2 =
            [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response)
            -> void {
        RCLCPP_INFO(this->get_logger(), "Turtle Created: %s",
                    response.get()->name.c_str());
        rclcpp::shutdown();
        };

        result = client->async_send_request(std::move(SpawnRequest2), callback2);

        
    }

} // namespace composition

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::p3_spawn)