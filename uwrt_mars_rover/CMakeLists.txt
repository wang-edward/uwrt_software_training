cmake_minimum_required(VERSION 3.8)
project(uwrt_mars_rover)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs)
find_package(std_msgs)

#include the 'include' directory
include_directories(include)

# add clear_node as a plugin
add_library(p1_clear SHARED
          src/p1_clear_turtles.cpp)
target_compile_definitions(p1_clear
        PRIVATE "COMPOSITION_BUILDING_DLL")        
        ament_target_dependencies(p1_clear
        "rclcpp"
        "rclcpp_components"
        "turtlesim"
        "std_msgs")
      rclcpp_components_register_nodes(p1_clear "composition::clear_node")
      set(node_plugins "${node_plugins}composition::clear_node;$<TARGET_FILE:p1_clear\n>")

# add circle_turtle_pub as a plugin
add_library(p2_circle SHARED
          src/p2_circle_turtle.cpp)
target_compile_definitions(p2_circle
        PRIVATE "COMPOSITION_BUILDING_DLL")        
        ament_target_dependencies(p2_circle
        "rclcpp"
        "rclcpp_components"
        "turtlesim"
        "geometry_msgs"
        "std_msgs")
      rclcpp_components_register_nodes(p2_circle "composition::circle_turtle_pub")
      set(node_plugins "${node_plugins}composition::circle_turtle_pub;$<TARGET_FILE:p2_circle\n>")

# add circle_turtle_pub as a plugin
add_library(p3_spawn SHARED
            src/p3_spawn_turtles.cpp)

target_compile_definitions(p2_circle
        PRIVATE "COMPOSITION_BUILDING_DLL")        
        ament_target_dependencies(p3_spawn
        "rclcpp"
        "rclcpp_components"
        "turtlesim"
        "std_msgs")
      rclcpp_components_register_nodes(p3_spawn "composition::spawn_node")
      set(node_plugins "${node_plugins}composition::spawn_node;$<TARGET_FILE:p3_spawn\n>")


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(TARGETS
        p1_clear
        p2_circle
        p3_spawn
        ARCHIVE DESTINATION lib
	      LIBRARY DESTINATION lib 
	      RUNTIME DESTINATION bin)
        
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
        ) 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
