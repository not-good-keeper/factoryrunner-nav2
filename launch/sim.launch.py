"""Depot world + default TurtleBot3 waffle + Nav2, with FactoryRunner's params and zone masks.

The stock tb3_simulation_launch.py does all of the robot and Nav2 bringup; this file only
swaps in the depot world and map and this package's params, and adds the costmap filter
mask servers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from factoryrunner.params_overlay import write_merged
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEPTH_CAMERA = ('/world/depot/model/turtlebot3_waffle/link/camera_link/sensor/'
                'intel_realsense_r200_depth/depth_image/points')


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1')


def _nav2(context, pkg, bringup, tb4_sim):
    """Nav2 bringup, with the opt-in overlays merged into the params when asked for."""
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    overlays = [os.path.join(pkg, 'config', 'overlays', f'{name}.yaml')
                for name in ('depth_costmap', 'velocity_stop') if _enabled(context, name)]
    if overlays:
        params = write_merged(params, overlays)

    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'world': os.path.join(tb4_sim, 'worlds', 'depot.sdf'),
            'map': os.path.join(bringup, 'maps', 'depot.yaml'),
            'params_file': params,
            'rviz_config_file': os.path.join(pkg, 'config', 'factoryrunner.rviz'),
            # Gazebo (-8, 0) is where the depot map was recorded from, i.e. map (0, 0).
            'x_pose': '-8.0',
            'y_pose': '0.0',
            'headless': LaunchConfiguration('headless'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )]

    if _enabled(context, 'depth_costmap'):
        sim_time = {'use_sim_time': True}
        actions += [
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='depth_points_bridge',
                 arguments=[f'{DEPTH_CAMERA}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
                 remappings=[(DEPTH_CAMERA, '/camera/depth/points')], parameters=[sim_time]),
            Node(package='factoryrunner_vision', executable='depth_cloud_relay',
                 parameters=[sim_time]),
            # Where Gazebo actually renders the depth sensor: camera link plus sensor offset in
            # the robot model, and 0.226 m up as measured from floor points. The description's
            # own camera_depth_frame sits lower and is an optical frame, so it cannot be used.
            Node(package='tf2_ros', executable='static_transform_publisher',
                 arguments=['--x', '0.133', '--y', '-0.094', '--z', '0.226',
                            '--frame-id', 'base_footprint', '--child-frame-id', 'depth_sensor'],
                 parameters=[sim_time]),
        ]
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('factoryrunner')
    bringup = get_package_share_directory('nav2_bringup')
    tb4_sim = get_package_share_directory('nav2_minimal_tb4_sim')

    use_zones = LaunchConfiguration('use_zones')

    # Mask servers for the keepout lane and the CNC speed zone. Topics are absolute because
    # the costmaps live in their own namespaces and would otherwise resolve them relatively.
    sim_time = {'use_sim_time': True}
    zone_nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_mask_server',
            parameters=[sim_time, {
                'yaml_filename': os.path.join(pkg, 'maps', 'keepout_mask.yaml'),
                'topic_name': '/keepout_filter_mask',
            }],
            condition=IfCondition(use_zones),
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_filter_info_server',
            parameters=[sim_time, {
                'type': 0,
                'filter_info_topic': '/keepout_filter_info',
                'mask_topic': '/keepout_filter_mask',
                'base': 0.0,
                'multiplier': 1.0,
            }],
            condition=IfCondition(use_zones),
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='speed_mask_server',
            parameters=[sim_time, {
                'yaml_filename': os.path.join(pkg, 'maps', 'speed_mask.yaml'),
                'topic_name': '/speed_filter_mask',
            }],
            condition=IfCondition(use_zones),
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speed_filter_info_server',
            # type 1 = percentage limit; mask value v gives a limit of 100 - v percent
            parameters=[sim_time, {
                'type': 1,
                'filter_info_topic': '/speed_filter_info',
                'mask_topic': '/speed_filter_mask',
                'base': 100.0,
                'multiplier': -1.0,
            }],
            condition=IfCondition(use_zones),
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_zones',
            parameters=[sim_time, {
                'autostart': True,
                'node_names': [
                    'keepout_mask_server',
                    'keepout_filter_info_server',
                    'speed_mask_server',
                    'speed_filter_info_server',
                ],
            }],
            condition=IfCondition(use_zones),
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='False',
                              description='Run Gazebo without its GUI'),
        DeclareLaunchArgument('use_rviz', default_value='True'),
        DeclareLaunchArgument('use_zones', default_value='True',
                              description='Start the keepout and speed zone mask servers'),
        DeclareLaunchArgument('depth_costmap', default_value='False',
                              description='Add the depth camera as an obstacle source'),
        DeclareLaunchArgument('velocity_stop', default_value='False',
                              description='Use a stop field that follows the commanded motion'),
        OpaqueFunction(function=_nav2, args=[pkg, bringup, tb4_sim]),
        *zone_nodes,
    ])
