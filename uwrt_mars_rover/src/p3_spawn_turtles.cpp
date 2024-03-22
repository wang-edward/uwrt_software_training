#include "../include/uwrt_mars_rover/p3_spawn_turtles.hpp"

using namespace std::chrono_literals;

namespace composition{

    spawn_node::spawn_node(const rclcpp::NodeOptions & options)
        : Node("spawn_node", options) {
            // creating a client to call the server
            client_ = this->create_client<turtlesim::srv::Spawn>("spawn");

             // Populate the map
            for(size_t i = 0; i < turtles.size(); ++i) {
                turtle_description[turtles[i]] = turtle_loc[i];
            }
            //call spawn function
            spawn_turts();
        }

        void spawn_node::spawn_turts() {
    
            // send a request for all nodes
            for (std::string &turtle : turtles) {
                auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
                // Kill (param: name)
                request->name = turtle;
                request->x = turtle_description[turtle].x_pos;
                request->y = turtle_description[turtle].y_pos;
                request->theta = turtle_description[turtle].theta;

            
                // create callback to handle responce
                auto callback = [this, turtle](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) {
                    // fetch a responce from the service call and store it in output
                    auto output = response.get();
                    std::cout<<"Spawned: "<< turtle <<" Service Call Responce: "<<output<<std::endl;
                    // shutdown server
                    rclcpp::shutdown();
                };

                // get the result from server
                auto result = client_->async_send_request(request, callback);
            }

    }

} //namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(composition::spawn_node)