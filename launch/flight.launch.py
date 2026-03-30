import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_arming_client'),
        'config',
        'params.yaml'
    )

    node = Node(
        package='my_arming_client',
        executable='arming_client',
        name='global_flight_node',
        parameters=[config]
    )

    return LaunchDescription([node])
