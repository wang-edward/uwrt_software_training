#include "../include/uwrt_mars_rover/p1_clear_turtles.hpp"

using namespace std::chrono_literals;

namespace composition{

    clear_node::clear_node(const rclcpp::NodeOptions & options)
        : Node("clear_node", options) {
            // creating a client to call the server
            client_ = this->create_client<turtlesim::srv::Kill>("kill");

            // creating a callback timer
            timer_ = this->create_wall_timer(
                500ms, std::bind(&clear_node::kill, this));
        }

        void clear_node::kill() {
    
            // send a request for all nodes
            for (std::string &turtle : turtles) {
                auto request = std::make_shared<turtlesim::srv::Kill::Request>();
                // Kill (param: name)
                request->name = turtle;
            
            // create callback to handle responce
            auto callback = [this, turtle](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture response) {
                // fetch a responce from the service call and store it in output
                auto output = response.get();
                std::cout<<"Killed: "<< turtle<<" Service Call Responce: "<<output<<std::endl;
                // shutdown server
                rclcpp::shutdown();
            };

            // get the result from server
            auto result = client_->async_send_request(request, callback);
            }

    }

} //namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::clear_node)