import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():
    """Generate launch description with multiple components."""
    clear_container = ComposableNodeContainer(
            name='clear_container',
            namespace='clear',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='uwrt_mars_rover',
                    plugin='composition::clear_node',
                    name='p1_clear')
            ],
            output = 'screen'
    )

    circle_container = ComposableNodeContainer(
            name='circle_container',
            namespace='circle',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='uwrt_mars_rover',
                    plugin='composition::circle_turtle_pub',
                    name='p2_circle')
            ],
            output = 'screen'
    )

    spawn_container = ComposableNodeContainer(
            name='spawn_container',
            namespace='spawn',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='uwrt_mars_rover',
                    plugin='composition::spawn_node',
                    name='p3_spawn')
            ],
            output = 'screen'
    )
    
    turtlesim = Node(
          package='turtlesim',
          executable='turtlesim_node',
          name='sim',
          output='screen',
    )
    after_turtlesim_start = RegisterEventHandler(
          OnProcessStart(
                target_action = turtlesim,
                on_start = clear_container,
          )
    )
    after_turtle1_clear = RegisterEventHandler(
        OnExecutionComplete(
            target_action=clear_container,
            on_completion=[
                LogInfo(msg='Clear finished'),
                spawn_container
            ],
        )
    )
    after_spawn = RegisterEventHandler(
        OnExecutionComplete(
            target_action=spawn_container,
            on_completion=[
                LogInfo(msg='Spawn finished'),
                circle_container
            ],
        )
    )

    process_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=turtlesim,
            on_exit=[
                LogInfo(msg=(EnvironmentVariable(name='USER'),
                        ' closed the turtlesim window')),
                EmitEvent(event=Shutdown(
                    reason='Window closed'))
            ]
        )
    )
    shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[LogInfo(
                msg=['Launch was asked to shutdown: ',
                    LocalSubstitution('event.reason')]
            )]
        )
    )

    return launch.LaunchDescription([turtlesim, after_turtlesim_start, after_turtle1_clear, after_spawn, process_exit, shutdown])
