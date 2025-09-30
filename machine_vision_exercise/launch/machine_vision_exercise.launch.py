from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the moveit2.launch.py file from ros2srrc_launch
    moveit_launch_file = os.path.join(
        get_package_share_directory('ros2srrc_launch'),
        'moveit2',
        'machine_vision.launch.py'
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            'package': 'ros2srrc_ur5',
            'config': 'ur5_7'
        }.items()
    )

    pcl_filter_node = Node(
        package='pcl_filter',
        executable='pcl_filter_server',
        name='pcl_filter_server',
        output='screen'
    )

    return LaunchDescription([
        moveit_launch,
        pcl_filter_node
    ])
