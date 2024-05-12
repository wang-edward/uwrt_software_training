import launch

import launch.event_handlers
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():
    turtlesim_launch = Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node'
    )
    
    turtle_init_container = ComposableNodeContainer(
        name='init_node_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='turtlesim_cpp_pkg',
                plugin='composition::p1_clear',
                name='p1_clear',
                ),
            ComposableNode(
                package='turtlesim_cpp_pkg',
                plugin='composition::p3_spawn',
                name='p3_spawn',
                ),
        ],
        output='screen',
    )
    
    turtle_circle_container = ComposableNodeContainer(
        name = 'circle_node_container',
        namespace='',
        package = 'rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='turtlesim_cpp_pkg',
                plugin='composition::p2_circle',
                name='p2_circle',
                ), 
        ],
        output = 'screen',
    )
    

    on_turtlesim_start = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessStart(
            target_action=turtlesim_launch,
            on_start = turtle_init_container
        )
    )
    on_clear_complete = launch.actions.RegisterEventHandler(
            launch.event_handlers.OnExecutionComplete(
                target_action=turtle_init_container,
                on_completion=turtle_circle_container
            )
        )
    

    
    return launch.LaunchDescription([turtlesim_launch,on_turtlesim_start, on_clear_complete])
    
    

