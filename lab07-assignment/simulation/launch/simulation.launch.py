# Software License Agreement (BSD License 2.0)
#
# Copyright (c) 2023, Metro Robots
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Metro Robots nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # these are the arguments you can pass this launch file, for example paused:=true
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set "false" to run GZ Sim without GUI'
    )
    package_arg = DeclareLaunchArgument(
        'urdf_package',
        description='The package where the robot description is located',
        default_value='simulation'
    )
    model_arg = DeclareLaunchArgument(
        'urdf_package_path',
        description='The path to the robot description relative to the package root',
        default_value='urdf/my_rover.urdf.xacro'
    )   
    server_config_path = PathJoinSubstitution([
        FindPackageShare('simulation'), 
        'configs',
        'server.config'
    ])
    server_config_env = SetEnvironmentVariable(
        'GZ_SIM_SERVER_CONFIG_PATH', 
        server_config_path
    )
    world_file_path = PathJoinSubstitution([
        FindPackageShare('simulation'), 
        'worlds', 
        'default.sdf'
    ])

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Path to the world SDF file'
    )

    world = LaunchConfiguration('world')

    # bridge 설정 파일: simulation/configs/bridge.yaml
    bridge_config_path = PathJoinSubstitution([
        FindPackageShare('simulation'),
        'configs',
        'bridge.yaml'
    ])

    empty_world_launch = IncludeLaunchDescription(
        #PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 
                'launch', 
                'gz_sim.launch.py'
            ])),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
#            'pause': 'true',
            'gz_args' : [TextSubstitution(text='-r -v 1 '), world],
            'on_exit_shutdown' : 'true'
        }.items(),
    )

    description_launch_py = IncludeLaunchDescription(
        # PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        # launch_arguments={
        #     'urdf_package': LaunchConfiguration('urdf_package'),
        #     'urdf_package_path': LaunchConfiguration('urdf_package_path')}.items()
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('urdf_launch'),
                'launch', 
                'description.launch.py'
            ])),
            launch_arguments={
                'urdf_package': LaunchConfiguration('urdf_package'),
                'urdf_package_path': LaunchConfiguration('urdf_package_path')
            }.items()
    )

    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner = IncludeLaunchDescription(
        # package='gazebo_ros',
        # executable='spawn_entity.py',
        # name='urdf_spawner',
        # arguments=['-topic', '/robot_description', '-entity', 'robot', '-z', '0.5', '-unpause'],
        # output='screen',
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch', 
                'gz_spawn_model.launch.py'
            ])),
        launch_arguments={
                'world': 'default',
                'topic': '/robot_description',
                'entity_name': 'robot',
                'z': f'{0.5}',
            }.items()
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'config_file' : bridge_config_path,
            }
        ],
        output='screen',
    )

    camera_viewer_node = Node(
        package='simulation',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        package_arg,
        model_arg,
        world_arg,
        server_config_env,
        empty_world_launch,
        bridge_node,
        description_launch_py,
        urdf_spawner,
        camera_viewer_node,
    ])
