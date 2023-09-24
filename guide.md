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

