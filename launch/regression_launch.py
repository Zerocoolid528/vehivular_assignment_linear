# Import LaunchDescription to define the structure of the launch file
from launch import LaunchDescription

# Import Node to describe how a ROS 2 node should be launched
from launch_ros.actions import Node

# Main function that returns the launch description
def generate_launch_description():
    return LaunchDescription([
        # Launch the regression_node from the linear_regression_pkg package
        Node(
            package='linear_regression_pkg',     # Name of the ROS 2 package
            executable='regression_node',        # Name of the executable as defined in setup.py
            name='regression_node',              # Name to assign to the running node
            output='screen'                      # Print node output to the terminal screen
        )
    ])
