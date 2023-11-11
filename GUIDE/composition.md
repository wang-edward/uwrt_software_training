# In-Depth Guide to ROS2 Node Composition

## Introduction

ROS2 Composition is a design pattern that allows for multiple nodes to be combined into a single executable, referred to as a "component container". This approach offers several advantages over running nodes as separate processes, such as improved performance due to reduced inter-process communication (IPC) overhead, easier management of the application lifecycle, and potential benefits for real-time systems.

## Understanding ROS2 Nodes and Executors

In ROS2, a node is the fundamental entity that communicates with other parts of the system. Nodes can publish or subscribe to topics, provide or use services, and perform computation. The ROS2 executor is responsible for managing the callbacks of a node. There are several types of executors:

1. **Single-threaded Executor in ROS2**:

   In ROS2, when you use a single-threaded executor, your ROS2 application will handle all the incoming messages, service requests, and other tasks one at a time in a single sequence, just like if you were completing tasks from your to-do list one by one. This is simple and straightforward but might not be the fastest if you have a lot of tasks.
</br>

2. **Multi-threaded Executor in ROS2**:

   If you use a multi-threaded executor, ROS2 can handle different tasks simultaneously, similar to having a group of friends tackle different chores at the same time. This means your ROS2 application can process multiple messages or service requests in parallel, which can be much faster, especially if there's a lot to do at once.
</br>
   

3. **Static Single-threaded Executor in ROS2**:

   When your ROS2 application uses a static single-threaded executor, it's like you've optimized your single-person workflow because you know exactly what tasks you'll have and there won't be any surprises. This can make your ROS2 application run more efficiently, but it's less flexible if the number or types of tasks suddenly change.
</br>
   

4. **Custom Executors in ROS2**:

   With custom executors, you get to tailor exactly how your ROS2 application handles tasks. Maybe you have some messages that are more important than others, or some operations that should happen as quickly as possible. With a custom executor, you can write special rules that decide the priority and order of tasks, giving you control to fine-tune the performance of your application.


## Composition vs. Non-Composition

### Non-Composition Model

In the non-composition model, each node is a separate process with its own memory space, lifecycle, and IPC mechanisms. This model provides isolation and fault tolerance since a failure in one node doesn't directly affect others.


## Without Composition: Separate Processes

In the traditional approach without composition, each node is a separate process. Here’s how it works:

1. **Process Isolation**: Each node runs in its own process, which means it has its own memory space and is scheduled independently by the operating system.

2. **Inter-Process Communication (IPC)**: Nodes communicate with each other over the ROS2 middleware (e.g., DDS - Data Distribution Service), using publish/subscribe, services, or actions. Because they are separate processes, this communication must go through the operating system's IPC mechanisms.

3. **Resource Utilization**: Running each node as a separate process can result in higher overhead due to the context switching and IPC, especially if there are many small nodes.
  
4. **Fault Tolerance**: A crash in one node will not affect other nodes directly, which can be an advantage for fault isolation.

5. **Development and Debugging**: It's often easier to develop and debug nodes separately, as you can start, stop, and restart individual nodes without affecting others.


### Composition Model

In the composition model, multiple nodes (components) are loaded into a single process (container). The communication between components can bypass IPC, using direct function calls instead, which is more efficient.


## With Composition: Single Process

With composition, multiple nodes are loaded into a single process as components. Here’s the breakdown of this approach:

1. **Shared Memory Space**: All components share the same memory space, which means they can communicate directly without IPC, using in-process communication (essentially, function calls).

2. **Intra-Process Communication**: When nodes are composed together, ROS2 can use an optimized communication path that avoids serialization and deserialization of messages. This is much faster than IPC.

3. **Resource Utilization**: By sharing resources like memory and by avoiding the overhead of IPC, composed nodes can run more efficiently. This can be particularly important for real-time applications.

4. **Fault Tolerance**: Because all components run in the same process, a crash in one component can bring down the entire process. Therefore, this approach requires careful handling of exceptions and errors.

5. **Development and Debugging**: Debugging can be more complex since a problem in one component can affect all others. However, managing the application as a whole (starting, stopping, deploying) can be simpler.

6. **Dynamic Composition**: ROS2 allows for dynamic composition, where components are loaded and unloaded at runtime. This is managed by a component container, which exposes services for loading and unloading components without restarting the entire process.

7. **Real-time Performance**: For nodes that need to communicate with low latency or need to be tightly synchronized, composition can provide better performance.

8. **Lifecycle Management**: Composed nodes can still use lifecycle management, which allows for a fine-grained control of the state of each node, but this is now within the context of a shared process.
### Under the Hood Comparison

- **Without Composition**: There is an operating system boundary between each node, which leads to context switches and potential delays due to IPC overhead.
- **With Composition**: Components bypass the OS's IPC and communicate directly, which can significantly reduce latency and overhead.

The choice between composition and no composition depends on the specific requirements of your application, such as performance, fault tolerance, and ease of development. For large-scale and complex systems, a mixed approach is often used, where critical components that need to communicate frequently and with low latency are composed together, while less critical parts of the system are run as separate processes to benefit from process isolation.

## ROS2 Composition in Detail

### Components as Shared Libraries

In ROS2, components are compiled into shared libraries (Read appendix A for more on libraries and executables) rather than standalone executables. This allows a component container to dynamically load and unload these components at runtime.

### Executors and Thread Management

In a composed system, the executor handles the callbacks from all loaded components. Depending on the type of executor used, these callbacks may be handled in a single thread or by a pool of threads.

### Why No `main` Function?

In ROS2 composition, each component does not require a `main` function because the components are intended to be loaded into a container that already has a running context. The component container is responsible for initializing the ROS2 environment, creating an executor, and spinning the executor to handle callbacks. The shared libraries contain the node logic but rely on the container to be the entry point of the application.

## Creating a Composed ROS2 Application

### Step 1: Define Component Nodes

Define your nodes as C++ classes inheriting from `rclcpp::Node`. Here's an example of a simple node that publishes a string message:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node
{
public:
  Talker() : Node("talker")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), [this]() { publish_message(); });
  }

private:
  void publish_message()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world!";
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(Talker)
```


### Step 2: Register Component Nodes

Use the `RCLCPP_COMPONENTS_REGISTER_NODE` macro to register the node with the component system: 

Registering component nodes with the `RCLCPP_COMPONENTS_REGISTER_NODE` macro in ROS2 is a critical step because it enables the component container to recognize and manage these nodes at runtime. This registration process makes the nodes discoverable and loadable on demand, allowing for dynamic composition of the application. Without this registration, the nodes would not be part of the ROS2 component ecosystem, thus losing out on the modularity, flexibility, and efficiency that come with dynamic loading and unloading of components within a running system.

```cpp
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(Talker)
```

### Step 3:  Adding a Node to the CMakeLists.txt in ROS2

In a `CMakeLists.txt` file for a ROS2 package, you define how your package should be built. For ROS2 nodes that are meant to be loaded as components, you have to compile them into shared libraries rather than standalone executables. Here's how you would add a node to the `CMakeLists.txt` file for it to be compiled as a component:

1. **Define the library**: Tell CMake to create a shared library from your source files.
2. **Declare dependencies**: Specify any ROS2 packages or system libraries that your node depends on.
3. **Register the node as a component**: Use the `rclcpp_components_register_nodes` function to register the node as a component that can be loaded at runtime.

## Example CMakeLists.txt

Here's an example of what the relevant lines in the `CMakeLists.txt` file might look like for a node called `Talker`:

```cmake
# Specify the minimum version of CMake required to build this project
cmake_minimum_required(VERSION 3.5)

# Declare the name of the project or package
project(my_ros2_package)

# Find and load the build-time dependencies specified by 'ament_cmake'
find_package(ament_cmake REQUIRED)

# Find and load the 'rclcpp' package which provides ROS2 client library for C++
find_package(rclcpp REQUIRED)

# Find and load the 'rclcpp_components' package required for component nodes
find_package(rclcpp_components REQUIRED)

# Find and load the 'std_msgs' package which provides standard message types
find_package(std_msgs REQUIRED)

# Include the 'include' directory for C++ header files
include_directories(include)

# Declare a new library target called 'talker_component', compile it as a shared library from the specified source files
add_library(talker_component SHARED
  src/talker_component.cpp)

# Link the 'talker_component' library with its dependencies so that it knows which other libraries it needs to link against
ament_target_dependencies(talker_component
  rclcpp
  rclcpp_components
  std_msgs)

# Register the node class 'Talker' from the 'talker_component' library as a component that can be dynamically loaded
rclcpp_components_register_nodes(talker_component "my_ros2_package::Talker")

# Install the 'talker_component' library so that it can be loaded at runtime by the component container. Specify the destinations for the library files.
install(TARGETS
  talker_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Generate and install necessary CMake config files for your package so that other packages can find and use your package
ament_package()


```


In this `CMakeLists.txt`:

- `my_ros2_package` is the name of your ROS2 package.
- `talker_component` is the name of the shared library that you're creating.
- `src/talker_component.cpp` is the path to the source file for the `Talker` node class.
- `my_ros2_package::Talker` is the fully qualified class name of the node that you're registering.

Make sure you replace `my_ros2_package`, `talker_component`, and `src/talker_component.cpp` with the actual names and paths you're using in your project. After adding these lines, when you build your package with `colcon build`, it will create a shared library for the `Talker` node, which can then be dynamically loaded into a component container at runtime.

## Appendix A 

# ROS Node Execution Models

## Traditional ROS Node Model

In traditional ROS (both ROS1 and non-composed ROS2 applications), each node is a standalone executable. This means that each node has its own `main()` function, which is the entry point when the operating system starts running the program. Each of these executables initializes a ROS context and spins up an executor to handle callbacks and communication with other nodes.

## ROS2 Composition Model

In ROS2's composition model, the concept is different. Instead of compiling each node as a standalone executable, nodes are compiled as shared libraries (`.so` files on Unix-like operating systems or `.dll` files on Windows). These shared libraries don't have a `main()` function and aren't designed to be run on their own. Instead, they are loaded at runtime into a running process, which is typically the component container.

### Compilation to Shared Libraries

When you compile a ROS2 node as a shared library, you create a dynamically loadable piece of code. This code contains all the functions and classes that define the node, but it lacks the standalone executable's entry point (`main()` function). Instead of being an independent application, it's meant to be part of an application.

### Dynamic Loading

A component container is a special type of ROS2 executable that's designed to load and unload these shared library nodes dynamically. When it starts, the container sets up a ROS2 context and an executor, just like a standalone node would. However, it also exposes services that allow for the dynamic loading and unloading of components (the shared library nodes).

### At Runtime

Once the component container is running, it can be instructed (for example, via command-line tools or service calls) to load a component. It does this by loading the shared library into its process space. The container then initializes an instance of the component node class within the library and adds it to the executor. The node is now active and can publish or subscribe to topics, provide or use services, and so on.

### Unloading Components

Just as a container can load components dynamically, it can also unload them. When a component is unloaded, the container destroys the instance of the node class and unloads the shared library, freeing up resources.

### Benefits of Dynamic Loading

Dynamic loading has several advantages:

- **Efficiency**: Multiple nodes can share the same process and memory space, which can be more efficient than running many separate processes.
- **Flexibility**: Nodes can be added or removed from a running system without restarting the entire system.
- **Resource Sharing**: Nodes can easily share resources such as memory, which can be particularly advantageous for data that needs to be accessed by multiple nodes.
- **Reduced Overhead**: Because they share the same process, the nodes/components do not need to perform IPC to communicate with each other, reducing the overhead of communication.

### The reason we do this in CMakelist.txt

By compiling nodes as shared libraries and using a component container to manage them, ROS2 allows developers to create systems that are both modular and efficient. This is especially useful for large and complex robotic applications where performance is critical, and system downtime for updates or changes must be minimized.
