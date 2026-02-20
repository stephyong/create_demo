from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory  
import os

def generate_launch_description():

    params_file = os.path.join(                                        
        get_package_share_directory('ur16e_handover'),                 
        'config',                                                      
        'waypoints.yaml')                                             

    handover_node = Node(
        package="ur16e_handover",
        executable="handover", #node name 
        output="screen",
        parameters=[params_file]                                       
    )

    return LaunchDescription([
        handover_node
    ])