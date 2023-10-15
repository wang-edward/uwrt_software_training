cmake_minimum_required(VERSION 3.8)
project(software_training)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------------------
# find packages 
# -------------------------------------------------------------------
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

# -------------------------------------------------------------------
# include, generate interfaces, and setup plugins 
# -------------------------------------------------------------------
include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

#create resource which references the libraries in the binary bin
set(node_plugins "")

# -------------------------------------------------------------------
# P1
# -------------------------------------------------------------------
add_library(p1 SHARED
            src/p1_clear.cpp)

ament_target_dependencies(p1
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "geometry_msgs"
  "std_msgs")   
rclcpp_components_register_nodes(p1 "composition::p1_clear")
set(node_plugins "${node_plugins}composition::p1_clear;$<TARGET_FILE:p1>\n")

# -------------------------------------------------------------------
# P2
# -------------------------------------------------------------------
add_library(p2 SHARED
            src/p2_circle.cpp)

ament_target_dependencies(p2
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "geometry_msgs"
  "std_msgs")   
rclcpp_components_register_nodes(p2 "composition::p2_circle")
set(node_plugins "${node_plugins}composition::p2_circle;$<TARGET_FILE:p2>\n")

# -------------------------------------------------------------------
# P3
# -------------------------------------------------------------------
add_library(p3 SHARED
            src/p3_spawn.cpp)

ament_target_dependencies(p3
  "rclcpp"
  "rclcpp_components"
  "turtlesim"
  "std_msgs")   
rclcpp_components_register_nodes(p3 "composition::p3_spawn")
set(node_plugins "${node_plugins}composition::p3_spawn;$<TARGET_FILE:p3>\n")

# -------------------------------------------------------------------
# Install binary locations
# -------------------------------------------------------------------

install(TARGETS
  p1	
  p2
  p3

	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

# -------------------------------------------------------------------
# Launch file
# -------------------------------------------------------------------

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
