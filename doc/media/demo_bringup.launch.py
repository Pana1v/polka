"""
Capture bringup: polka_node + static transforms from the TIERS dataset extrinsics.

The Calibration.bag carries no TF. OS1 (`os_sensor`) is the base reference; the
other sensors are placed relative to it using the dataset README extrinsics.
`ros2 launch <this file> config_file:=<yaml>`.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# child_frame -> (x, y, z, roll, pitch, yaw)  [meters, radians] relative to os_sensor
EXTRINSICS = {
    'avia_frame': (0.149354, 0.0423582, -0.0524961, 3.13419, -3.13908, -3.13281),
    'mid360_frame': (0.125546, -0.0554536, -0.20206, 0.00467344, 0.0270294, 0.0494959),
    'camera_depth_optical_frame': (-0.172863, 0.11895, -0.101785, 1.55222, 3.11188, 1.60982),
}


def generate_launch_description():
    cfg = LaunchConfiguration('config_file')
    actions = [
        DeclareLaunchArgument('config_file'),
        Node(package='polka', executable='polka_node', name='polka',
             output='screen', parameters=[cfg, {'use_sim_time': True}]),
    ]
    for child, (x, y, z, roll, pitch, yaw) in EXTRINSICS.items():
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'tf_{child}', output='log',
            arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                       '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                       '--frame-id', 'os_sensor', '--child-frame-id', child]))
    return LaunchDescription(actions)
