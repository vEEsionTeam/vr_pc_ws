from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('gazebo_sim')
    urdf_path = os.path.join(pkg_path, 'urdf', 'head.urdf')
    world_path = os.path.join(pkg_path, 'worlds', 'cafe.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gazebo', world_path,
                '--verbose',
                '-s', 'libgazebo_ros_factory.so',
                '-s', 'libgazebo_ros_init.so'
            ],
            output='screen'
        ),
        # Robot state publisher (so TFs and frames are published)
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
            arguments=['-entity', 'head_model', '-file', urdf_path,
                '-x', '0', '-y', '0', '-z', '1.5',
                #'-R', '0', '-P', '0', '-Y', '-1.5708'  # 180 degrees in radians
                '-R', '0', '-P', '0', '-Y', '1.5708'  # 180 degrees in radians          
                       
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


