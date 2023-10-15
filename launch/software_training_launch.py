# from launch import LaunchDescription
# from launch_ros.actions import LoadComposableNodes, Node
# from launch_ros.descriptions import ComposableNode

import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
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

import random

def generate_launch_description():
    clear_container = ComposableNodeContainer(
            name = 'clear_container',
            namespace = 'clear',
            package = 'rclcpp_components',
            executable = 'component_container',
            composable_node_descriptions = [
                ComposableNode(
                    package ='software_training',
                    plugin = 'composition::p1_clear',
                    name = 'p1_clear'),
            ],
            output = 'screen',
    )
    spawn_container = ComposableNodeContainer(
            name = 'spawn_container',
            namespace = 'spawn',
            package = 'rclcpp_components',
            executable = 'component_container',
            composable_node_descriptions = [
                ComposableNode(
                    package = 'software_training',
                    plugin = 'composition::p3_spawn',
                    name = 'p3_spawn'),
            ],
            output='screen',
    )
    demo_container = ComposableNodeContainer(
        name = 'demo_container',
        namespace = 'demo',
        package = 'rclcpp_components',
        executable = 'component_container',
        composable_node_descriptions = [
            ComposableNode(
                package = 'software_training',
                plugin = 'composition::p2_circle',
                name = 'p2_circle',
            ), ComposableNode(
                package = 'software_training',
                plugin = 'composition::p4_reset',
                name = 'p4_reset',
            ), ComposableNode(
                package = 'software_training',
                plugin = 'composition::p5_distance',
                name = 'p5_distance',
            ), ComposableNode(
                package = 'software_training',
                plugin = 'composition::p6_waypoint',
                name = 'p6_waypoint',
            ),
        ],
        output = 'screen',
    )
    turtlesim = Node(
        package = 'turtlesim',
        executable = 'turtlesim_node',
        name = 'sim',
        output = 'screen',
    )

    rand_x = "{:.2f}".format(random.uniform(1.0, 9.0))
    rand_y = "{:.2f}".format(random.uniform(1.0, 9.0)) 
    send_goal = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' action send_goal /p6_waypoint software_training/action/Waypoint "{x: ',
            rand_x,
            ' , y: ',
            rand_y,
            ' }" '
        ]],
        shell=True
    )

    on_turtlesim_start = RegisterEventHandler(
        OnProcessStart(
            target_action = turtlesim,
            on_start = clear_container,
        )
    )
    on_clear = RegisterEventHandler(
        OnExecutionComplete(
            target_action = clear_container,
            on_completion=[
                LogInfo(msg = 'clear finished, spawn running'),
                spawn_container
            ],
        )
    )
    on_spawn = RegisterEventHandler(
        OnExecutionComplete(
            target_action = spawn_container,
            on_completion = [
                LogInfo(msg = 'spawn finished, demo running'),
                demo_container,
                TimerAction(
                    period=2.0,
                    actions=[send_goal],
                )
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

    return launch.LaunchDescription([turtlesim, on_turtlesim_start, on_clear, on_spawn, process_exit, shutdown])


    #  nodes
    # event handler
