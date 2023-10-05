# CMake tutorial

## Setup commands
```CMake
cmake_minimum_required(VERSION 3.8) # sets the CMake minimum version
project(software_training) # sets the project name 
```

- These commands are required in every CMake file
- project() also sets the PROJECT_NAME variable

## find_package()
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

## include_directories()
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

## generate_interfaces()
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

## ament_target_dependencies

syntax:
```CMake
ament_target_dependencies( <name> 
    <INTERFACE|PUBLIC|PRIVATE> [items1...]
    [<INTERFACE|PUBLIC|PRIVATE> [items2...] 
)
```

example:
```CMake
ament_target_dependencies( p1
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "geometry_msgs"
  "std_msgs")   
```
- binds the dependencies to p1
- ament_target_dependencies is a macro that runs a series of CMake functions under the hood
- [More info](https://www.reddit.com/r/ROS/comments/i6r6a4/target_link_libraries_vs_ament_target_dependencies/)

## rclcpp_components_register_nodes()

```CMake
rclcpp_components_register_nodes(p1 "composition::p1_clear")
```
- Registers the rclcpp component p1 with the ament resource index
- The index is composition::p1_clear
- There is a similar C macro that you need to include at the end of your .cpp files

## RCLCPP_COMPONENTS_REGISTER_NODE (not a CMake command)
```c
#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(composition::action_turtle)
```

## set(node_plugins) (again)
```CMake
set(node_plugins "${node_plugins}composition::p4_reset;$<TARGET_FILE:p4\n>")
```

- After we register the components with the Ament index, we need to update the node_plugins variable
- See above for more info

## install()
note: install is super complicated so you don't need to understand it fully
```CMake
install(TARGETS
  p1	
  p2
  p3
  p4
  p5
  p6 

	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin
)

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)
```
- generates installation rules for a project
- install(TARGETS)
    - installs the target executables(p1, p2 etc.) to different destinations
try (from the root of your directory):
```bash
ls install/software_training/lib
ls install/software_training/lib | grep p3
```
- install(DIRECTORY)
    - in this example, I made a folder that held launch files
    - I want these to be available when i build my package, so:
    - install copies the folder launch into the destination share/${project_name}, making it available
- [More info](https://cmake.org/cmake/help/latest/command/install.html)

## ament_package()
```CMake
ament_package()
```

- should **always** be the last command in your CMakeLists.txt
- a macro to do main project setup
    - installs the package.xml
    - registers the package with the ament index
    - The argument to project will be the package name and **must be identical** to the package name in the package.xml
