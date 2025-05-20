from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_path = get_package_share_directory('gazebo_sim')
    
    # Launch argument for world name
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='Warehouse.world',
        description='Name of the world file inside the worlds directory'
    )
    world = LaunchConfiguration('world')
    
    urdf_path = os.path.join(pkg_path, 'urdf', 'head.urdf')
    
    world_path = PathJoinSubstitution([
        pkg_path,
        'worlds',
        world
    ])

    return LaunchDescription([
        world_arg,

        ExecuteProcess(
            cmd=[
                'gazebo',
                world_path,
                '--verbose',
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_init.so'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'head_model',
                '-file', urdf_path,
                '-x', '0', '-y', '0', '-z', '1.5',
                '-R', '3.1416',  # 180 degrees in radians
                '-P', '0',
                '-Y', '0'
            ],
            output='screen'
        ),

        Node(
            package='gazebo_sim',
            executable='sphere_mover',
            name='sphere_mover',
            output='screen'
        ),

        Node(
            package='vr_camera',
            executable='vr_node',
            name='vr_node',
            output='screen'
        )
    ])
