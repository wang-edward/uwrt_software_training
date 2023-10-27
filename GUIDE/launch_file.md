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


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```
