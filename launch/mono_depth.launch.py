import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    pkg_dir = get_package_share_directory('mono_depth')
    list = [
        Node(
            package='mono_depth',
            executable='mono_depth',
            namespace='',
            # Azure Kinect
            # remappings=[('image_raw', '/rgb/image_raw'),
            #             ('camera_info', '/rgb/camera_info'),],
            # theta v
            remappings=[('image_raw', '/thetav/image_raw'),
                        ('camera_info', '/thetav/camera_info'),],
            output="screen",
            respawn=True,
        ),
        # launch_ros.actions.ComposableNodeContainer(
        #     name='container',
        #     namespace='',
        #     package='rclcpp_components',
        #     executable='component_container',
        #     composable_node_descriptions=[
        #         # Driver itself
        #         launch_ros.descriptions.ComposableNode(
        #             package='depth_image_proc',
        #             plugin='depth_image_proc::PointCloudXyzNode',
        #             name='point_cloud_xyz_node',
        #             remappings=[('image_rect', '/mono_depth/depth'),
        #                         ('camera_info', '/mono_depth/camera_info'),
        #                         ('points', '/mono_depth/point_cloud')],
        #         ),
        #     ],
        #     output='screen',
        # ),
    ]

    return LaunchDescription(list)