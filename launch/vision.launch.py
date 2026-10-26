"""Bridge the Waffle's stock depth camera into ROS and run the depth obstacle detector.

ros2 launch factoryrunner vision.launch.py      (alongside sim.launch.py)

The camera is part of the default robot model and renders in Gazebo already; Nav2's TB3
launch just does not bridge it. Nothing here changes the robot or what Nav2 does.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

CAMERA = '/world/depot/model/turtlebot3_waffle/link/camera_link/sensor/intel_realsense_r200_depth'


def generate_launch_description():
    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_camera_bridge',
            arguments=[
                f'{CAMERA}/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'{CAMERA}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
            remappings=[
                (f'{CAMERA}/depth_image', '/camera/depth/image_raw'),
                (f'{CAMERA}/camera_info', '/camera/depth/camera_info'),
            ],
            parameters=[sim_time],
        ),
        Node(
            package='factoryrunner_vision',
            executable='depth_obstacle_detector',
            parameters=[sim_time],
            output='screen',
        ),
    ])
