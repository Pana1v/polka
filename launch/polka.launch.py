# Copyright 2025 Panav Arpit Raaj <praajarpit@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('polka')
    default_config = os.path.join(pkg_dir, 'config', 'example_params.yaml')

    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    node_params = [config_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        # Default false for live sensor data. Set true to replay a rosbag, and play the
        # bag with the --clock flag (ros2 bag play <bag> --clock) so the staleness check
        # compares against bag time rather than wall time:
        #   ros2 launch polka polka.launch.py use_sim_time:=true
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # The terminal dashboard is always on. The node logs to file so its output
        # does not fight the TUI; the polka_monitor TUI (below) owns this terminal.
        # The same dashboard is available standalone in any other terminal:
        #   ros2 run polka polka_monitor
        Node(
            package='polka',
            executable='polka_node',
            name='polka',
            output={'both': 'log'},
            parameters=node_params,
        ),
        # ros2 launch pipes child stdio (no controlling TTY, no stdin), which curses
        # needs. Re-attach the monitor to the real terminal via /dev/tty so it can draw
        # and read keys. Standalone `ros2 run polka polka_monitor` needs none of this.
        # Exec the installed script directly (not through `ros2 run`, which forks its
        # own child process) so this action tracks a single PID launch can reliably
        # signal on shutdown - going through ros2 run left an orphaned polka_monitor
        # process behind on Ctrl-C, since launch's SIGINT/SIGTERM never reached the
        # grandchild it spawns.
        ExecuteProcess(
            cmd=['bash', '-c',
                 'exec "%s" --node polka </dev/tty >/dev/tty 2>&1' % os.path.join(
                     get_package_prefix('polka'), 'lib', 'polka', 'polka_monitor')],
            output='screen',
            additional_env={'FORCE_COLOR': '1'},
        ),
    ])
