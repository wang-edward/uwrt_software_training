# uwrt_software_training

This is a template + guides to working through the UW Robotics Software Onboarding

The goal of the onboarding is to get practice using ROS2 in c++. You'll do this through the turtlesim package, which simulates a robot.

![turtlesim pic](https://camo.githubusercontent.com/40caaa9f7bf024a5e9ad2ffcf89deb248029440af99aa0bc8cd371789dcfba5b/68747470733a2f2f692e6779617a6f2e636f6d2f39373465363765333834333162313063393938356339623033336565643537372e676966)

This code covers core concepts of ROS2 such as 
- components, topic statistic, callback groups, extending rclcpp::Node,
- pubs/subs, services/clients, action servers, and multi-threaded callbacks.

**Write 3 components that do the following: (Note: Some of the components do not need to be made in the following order)**
1. Clears any existing turtles
2. Create a component that moves 'turtle1' in a circular motion
3. Spawns a turtle named "stationary_turtle" at x = 5, y = 5
   Spawns a second turtle named "moving_turtle" at x = 25, y = 10

Lastly, create a launch file that will start up all the components and the turtlesim node (configure the parameters of the turtlesim node to however you see fit). Ensure that the turtlesim node is launched first as the other components are dependent upon it. 

## Instructions
- To start, clone this repository into the /src folder of your ros2_workspace
- This repo comes with a complete CMake file and templates for each question
- **Get started with the [GUIDE](https://github.com/wang-edward/uwrt_software_training/blob/main/GUIDE/guide.md)**
