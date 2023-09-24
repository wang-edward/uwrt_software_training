# UWRT Onboarding Guide

A guide designed to help get through the UW Robotics Team Onboarding.

General format:
- The guide will progressively reveal the solution: general strategy -> code structure -> full solution.
- Try to spend at least 5 minutes understanding each stage before moving to the next.
- Reading the linked articles is a good idea (especially configuring your workspace)
- Extra reading sections aren't necessary

## Introduction
The onboarding uses the turtlesim package to mimic a real life robot.

Throughout the onboarding, you'll write nodes that perform certain tasks to turtles within the sim. For example, moving them around or spawning and killing them.

## P1

Create a node that clears any existing turtles when it gets run.

### Prerequisite reading
[How to setup your ROS2 workspace with colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

[Understanding how nodes work](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

<details> 
  <summary>
    What architecture is best suited for this task? (Topic, Service / Client, Action)

  </summary>
    The best architecture to use is the service / client. 
</details>


<details>
    <summary>
    What are the benefits and drawbacks of each architecture? When would you use them?
    </summary>
    The main reason is that killing the turtles is a discrete message, so you call it on demand rather than continuously.
    A topic would be good for continuous messages, and an action server would be good for continuous messages that are controlled by discrete messages.
</details>


### General strategy

When you run turtlesim, a server is created that handles requests for the turtles
We want to make a client that sends requests to this server that tells it to kill each turtle.

### Code structure

#### Class
```cpp
public: 
clear_turtles(const rclcpp::NodeOptions &options);

private:
// the client to send requests with
rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client;

// all the turtles
std::vector<std::string> turtle_names = {"moving_turtle", "stationary turtle"};

// function that sends the kill requests
void kill();
```

#### Implementation
```cpp
constructor()
    // initialize the client
    // run the kill command

kill()
    for (auto name : turtle_names) {
        // create a message that holds the kill request (get the type right)
        // set the name field of the message
        // create a callback (kind of difficult so you can copy paste)
        auto callback = [this](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture response) -> void {
                (void)response; // void the response since we don't need one
                RCLCPP_INFO(this->get_logger(), "Turtle  Killed");
                rclcpp::shutdown(); // kill this node
        };
        // send an asynchronous request with the request message and the callback as parameters
    }
```

### Full solution
[header file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/include/software_training_assignment/clear_turtles.hpp)
[cpp file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/src/clear_turtles.cpp)

## P2

Create a component that moves 'turtle1' in a circular motion.

### General idea
- Create a publisher that sends geometry_msgs::msg::Twist messages to the "/turtle1/cmd_vel" topic
- The Twist messages represent velocity (linear and angular)
- The builtin listener for turtle1 will pick it up (subscribes to the topic)
- We can make the turtle go in a circle by sending the same direction and angle to move in
    - i.e. move forward 1 unit, turn left 1 unit

### Code structure

#### Class

```cpp
// publisher
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

// callback timer
rclcpp::TimerBase::SharedPtr timer;

// struct to hold a direction
struct Triple {
    float x, y, z;
}

// structs to hold the constant direction messages
Triple linear{1, 0, 0};
Triple angular{0, 0, 1};
```

#### Implementation 

```cpp
// create a publisher callback

constructor()
    // instantiate the publisher

    // create the publisher callback
    auto publisher_callback = [this](void) -> void {
        // instantiate the <geometry_msgs::msg::Twist> message
        // fill it out with the constant direction messages above
        // publish the message (this->publisher->publish)
    }

    // create a timer to run the callback every n milliseconds
```

### Full solutions (a bit small differences in structure)
[Header file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/include/software_training_assignment/turtle_circle_publisher.hpp)
[cpp file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/src/turtle_circle_publisher.cpp)

## P3
Spawn a turtle named "stationary_turtle" at x = 5, y = 5 

Spawns a second turtle named "moving_turtle" at x = 25, y = 10

### General idea
Create a client that sends <turtlesim::srv::Spawn> messages to the turtlesim server

### Code structure
#### Class
```cpp
public:
    spawn_turtle_nodelet(const rclcpp::NodeOptions &options);
private:
    
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client;
    rclcpp::TimerBase::SharedPtr timer;
    
    static const unsigned int NUMBER_OF_TURTLES{2};

    typedef struct turtle_info {
        float x_pos;
        float y_pos;
        float rad;
    } turtle_info;

    std::vector<string> turtle_names{"stationary_turtle", "moving_turtle"};
    std::vector<turtle_info> turtle_bio{{.x_pos = 5, .y_pos = 5, .rad = 0},
                                        {.x_pos = 25, .y_pos = 10, .rad = 0}};

    // map of turtle name to turtle information
    std::map<std::string, turtle_info> turtle_description;

    void spawn_turtle();
```

#### Implementation
```cpp
constructor()
    // initialize client
    // map names to bios
    spawn_turtle();

spawn_turtle()
    for each in turtle_names {
        // create a request message of type <turtlesim::srv::Spawn::Request>
        // fill out the request: (name, x, y, theta)
        // create a callback (look at previous examples)
            // callback should print info of turtle and then shutdown
        // send the async request (look at previous)
    }
```

### Full solution
[header_file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/include/software_training_assignment/spawn_turtle_nodelet.hpp)

[cpp file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/src/spawn_turtle_nodelet.cpp)

## P4
Create a service that resets the "moving_turtle" to its starting position. The service response should be whether or not it was successful.

### Prerequisite reading
- [Custom interface files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

### General idea
- Create a client to send the teleport request to the turtlesim server
- Create a service to receive the success message from the turtlesim server
    - Create a custom message to hold the success message

### Code structure

#### Custom message
```

---
bool success
```

#### CMake
```CMake
# custom services and messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/ResetMovingTurtle.srv"
	DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
	)
ament_export_dependencies(rosidl_default_runtime)
```

#### Class
```cpp
public:
    reset_moving_turtle(const rclcpp::NodeOptions &options);

private:
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client;
    // timer for running 
    rclcpp::TimerBase::SharedPtr timer;
    // create service that will reset turtle to starting point
    rclcpp::Service<software_training_assignment::srv::ResetMovingTurtle>::SharedPtr service;


    std::string turtle_to_move = "moving_turtle";
    float x_coord = 24.0f;
    float y_coord = 10.0f;
    float theta_coord = 0.0F;

    void service_callback(
        const std::shared_ptr<software_training_assignment::srv::ResetMovingTurtle::Request> request,
         std::shared_ptr<software_training_assignment::srv::ResetMovingTurtle::Response> response);
```

#### Implementation
```cpp
constructor()
    // initialize client
    // initialize service, pass in service_callback()
service_callback()
    // void cast the request
    // create the request message
    // fill out the request fields (x, y, theta)
    // create a response callback for the client
    callback {
        // void the response
        // log some success message 
    }

    // send the client request
    // set the service response to true (from the parameters)
```

#### Full solution
- [header file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/include/software_training_assignment/reset_moving_turtle.hpp)
- [cpp file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/src/reset_moving_turtle.cpp)
- [srv file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/srv/ResetMovingTurtle.srv)

## P5

Create a publisher that publishes a custom msg. 

This custom msgs should have 3 float fields that correspond with the x and y distances of "stationary_turtle" to "moving turtle", as well as the distance between the two turtles.

### Prerequisite Reading
- [Callback groups](https://docs.ros.org/en/humble/Concepts/About-Executors.html#callback-groups)

### General idea
- Create a custom message that holds the required data
- Create 2 subscribers to read the positions of each data
- Calculate the distance between them
- Publish the custom message

### Code structure

#### msg file
```cpp
float64 x_pos
float64 y_pos
float64 distance
```

#### CMake
```CMake
# custom services and messages and actions
rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/ResetMovingTurtle.srv"
	"msg/Software.msg"
	"action/Software.action"
	DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
	)
ament_export_dependencies(rosidl_default_runtime)
```

#### Class
```cpp
public:
    turtle_publisher(const rclcpp::NodeOptions &options);

private:
    // position subscribers
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr stationary_turt_sub;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr moving_turt_sub;

    // turtle publisher with custom message
    rclcpp::Publisher<software_training_assignment::msg::Software>::SharedPtr publisher;

    // timer for publisher callback
    rclcpp::TimerBase::SharedPtr timer;

    // callback groups - really just threads to run callbacks
    rclcpp::CallbackGroup::SharedPtr callbacks;

    float x_stationary_turt;
    float y_stationary_turt;

    float x_moving_turt;
    float y_moving_turt;

    float total_distance;
```

#### Implementation
```cpp
constructor()
    // create a callback for the stationary position
    callback {
        // fill out stationary x and y
    }

    // create a callback for the moving position
    callback {
        // fill out moving x and y
    }


    // create the custom publisher callback
    callback {
        // get x and y distance from stationary and moving variables
        // instantiate a custom message
        // fill out the custom message fields (x, y, distance)
            // how do you calculate distance between 2 points?
        / publish the message
    }

    // look at the solution for this next part
        // instantiate callback group
        // create callback thread
        // create custom name for topic
        // set callback topic to the custom name 

    // instantiate stationary subscriber
    // instantiate moving subscriber
    // instantiate publisher
    // set timer for publisher
```

### Full solution
- [Header file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/include/software_training_assignment/turtle_publisher.hpp)
- [cpp file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/src/turtle_publisher.cpp)
- [msg file](https://github.com/keyonjerome/uwrt_software_training_challenge/blob/master/software_training_assignment/msg/Software.msg)

## P6

Create an action that moves "moving_turtle" to a waypoint in a straight line 
- publish geometry_msgs/Twist msgs to turtleX/cmd_vel.

The action's goal command is an absolute position of the waypoint. 
- Feedback is distance to the goal
- result is the time it took to reach the goal. 
- You should define a custom action file.

### Prerequisite reading
- [Understanding actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

### General idea
- Create a publisher to send commands to the turtle
- Create a subscriber to get the turtle's position



