# CMake tutorial

## Setup commands
```CMake
cmake_minimum_required(VERSION 3.8) # sets the CMake minimum version
project(software_training) # sets the project name 
```

- These commands are required in every CMake file
- project() also sets the PROJECT_NAME variable

## Find packages
```CMake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
```
- These packages are software that is already installed on your computer (through apt)
- These commands tell the system to search through certain directories (like /share) to find packages
  - This works for all CMake projects, including ones you find on github, not just ROS2 packages!

Try:
```bash
ls /opt/ros/galactic/share
ls /opt/ros/galactic/share | grep turtlesim
```

```CMake
include_directories(include)
```
- Tells CMake to include files in the software_training/include folder
  - Your includes should go in software_training/include/software_training
- Allows you to do:
```cpp
#include "software_training/my_code.hpp" // instead of
#include "include/software_training/my_code.hpp"
```

```CMake
rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/Reset.srv"
  "msg/Distance.msg"
  "action/Waypoint.action"
	DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
	)
ament_export_dependencies(rosidl_default_runtime)
```
- Generates message and server files for your .srv, .msg, and .action files
  - DEPENDENCIES
    - interface files (i.e. .msg) can be composed of subinterfaces
    - sometimes you will want to include a pre-existing interface (defined by a package you installed)
    - Thus, you need to tell CMake which packages to look at.

## node_plugins

```CMake
set(node_plugins "")
```
- Resets the node_plugins variable
- Used for composition-based makefiles, stores a list of components that are used when generating an executable
- [More info](https://answers.ros.org/question/354631/what-does-setnode_plugins-actually-do/)

## add_library()

syntax:
```CMake
add_library(<name> [STATIC | SHARED | MODULE]
            [EXCLUDE_FROM_ALL]
            [<source>...])
```

example:
```CMake
add_library(p1 SHARED
            src/p1_clear.cpp)
```
- creates a library target called p1 to be built from the file src/p1_clear.cpp
- the <name> field has to be unique
- [More info on STATIC | SHARED | MODULE](https://cmake.org/cmake/help/latest/command/add_library.html#id2)

## target_compile_definitions
```cpp
target_compile_definitions(p1 PRIVATE
"SOFTWARE_TRAINING_BUILDING_DLL")
```
