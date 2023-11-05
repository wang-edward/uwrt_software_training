# Launch files

## Introduction

Launch files are used to automate tasks. Writing code in components allows us to compartmentalize our code, but makes running everything (in the correct order) tedious.

Launch files are written in python.

In order to learn the syntax, We will build an example launch file that 

## Structure
A typical launch file consists of a function called generate_launch_description().
This function should return a LaunchDescription object, which we'll see later

```py
def generate_launch_description():
  ...
  return launch.LaunchDescription( ... stuff ... )
```

Within the generate_launch_description() function, we create different launch objects, which are added to the LaunchDescription and passed to the user.

## Imports

We primarily import from 2 : launch and launch_ros. 
launch is a more general purpose library, while launch_ros adds some ros-specific concepts.
You can also import other python libraries, like random in order to help test different functionality of your programs.
```py
from launch import LaunchDescription
from launch_ros.actions import Node
```

## Composable Nodes

As you know, nodes are the basic components of any ROS system. 
As such, it's useful to be able to launch them.
We launch nodes by initializing a node object and adding it to the launch description.


### Composable properties
- A node has some properties
```python
myAwesomeNode = ComposableNode(
  package = 'software_training',
  plugin = 'composition::p1_clear',
  name = 'p1_clear',
)
```
You can see how the data definition matches the system
To run a regular (non-composable) node, you would typically do:
```bash
ros2 run software_training p1_clear
```
To run a composable node, it would be:
```md
ros2 component standalone software_training composition::p1_clear
```

Why is it that composable nodes have so much more stuff?

## How does composition work
Composable nodes need containers in order to be run.
- This makes sense! they have no main function, so how can they become an executable by themselves?

This brings us into our next topic: ComposableNodeContainer

## ComposableNodeContainer

A ComposableNodeContainer wraps a ComposableNode and allows it to be run

### ComposableNodeContainer datafields
```python
clear_container = ComposableNodeContainer(
  name = 'clear_container',
  namespace = 'clear',
  package = 'rclcpp_components',
  executable = 'component_container',
  composable_node_description = [
    ComposableNode(
      package = 'software_training',
      plugin = 'composition::p1_clear',
      name = 'p1_clear'),
  ],
  output = 'screen',
)
```

- name: friendly name
- namespace: friendly namespace
- package: gets the container source, should always be 'rclcpp_components'
- executable: same as package, always set to 'component_container'
- composable_node_description:
  - we nest the ComposableNode we defined above
- output: screen
  - display standard output onto the screen instead of in log files

## Event Handlers
- Sometimes we want to run Nodes based on events that happen
- There are 5 types of Event Handlers that we'll go over
  - OnProcessStart
  - OnProcessIO
  - OnExecutionComplete
  - OnProcessExit
  - OnShutdown

## OnProcessStart
- Registers a callback function that is executed when target_action starts. 
```python
on_start = RegisterEventHandler(
    OnProcessStart(
        target_action=turtlesim_node,
        on_start=[
            LogInfo(msg='Turtlesim started, spawning turtle'),
            spawn_turtle
        ]
    )
),

```
- in this case, executes spawn_turtle when the turtlesim node starts.

## OnProcessIO 
- registers a callback function that is executed when the target action writes to its standard output. 

```python
on_io = RegisterEventHandler(
  OnProcessIO(
    target_action = spawn_turtle,
    on_stdout = lambda event: LogInfo(
      msg = 'Spawn request says "{}"'.format(
        event.text.decode().strip()
      )
    )
  )
),
```
- logs the result of the spawn request.

## OnExecutionComplete
- registers a callback function that is executed when the spawn_turtle action completes
```python
RegisterEventHandler(
  OnExecutionComplete(
    target_action=spawn_turtle,
    on_completion=[
      LogInfo(msg='Spawn finished'),
      change_background_r,
      TimerAction(
        period=2.0,
        actions=[change_background_r_conditioned],
      )
    ]
  )
),
```
- logs a message to the console and executes the change_background_r and change_background_r_conditioned actions when the spawn action completes
- note: you can also pass a list of Nodes to on_completion
  - Here, we pass LogInfo (which prints to stdout), as well as the spawn_container node

## OnProcessExit
- registers a callback function that is executed when the turtlesim node exits. 
```python
process_exit = RegisterEventHandler(
    OnProcessExit(
        target_action=turtlesim_node,
        on_exit=[
            LogInfo(msg=(EnvironmentVariable(name='USER'),
                    ' closed the turtlesim window')),
            EmitEvent(event=Shutdown(
                reason='Window closed'))
        ]
    )
),
```
- logs a message to the console and executes the EmitEvent action to emit a Shutdown event when the turtlesim node exits. 
- the shutdown even means the launch process will shutdown when the turtlesim window is closed.
  - idea: don't want the launch to keep running if turtlesim is closed

## OnShutDown
- registers a callback function that is executed when the launch file is asked to shutdown. 
```python
on_shutdown = RegisterEventHandler(
  OnShutdown(
    on_shutdown = [
      LogInfo(
        msg = ['Launch was asked to shutdown: ',
          LocalSubstitution('event.reason')]
      )
    ]
  )
),
```
- logs a message to the console containing the shutdown reason, i.e:
  - closure of turtlesim window 
  - ctrl-c signal made by the user
