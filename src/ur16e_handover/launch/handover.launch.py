from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
def generate_launch_description():

    handover_node = Node(
        package="ur16e_handover",
        executable="handover", 
        output="screen" #prints the node's logs to the terminal 

    )

    return LaunchDescription([
        handover_node
    ])