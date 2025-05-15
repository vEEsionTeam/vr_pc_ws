import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

# Declare launch arguments
launch_args = [
    DeclareLaunchArgument(name="rviz_enable", default_value="true", description="Enable RViz node"),
    DeclareLaunchArgument(name="use_sim_time", default_value="false", description="Use simulation time"),
    DeclareLaunchArgument(name="urdf_file", default_value="rviz_head.urdf", description="URDF file to load"),
]

def launch_setup(context):

    # Get package path
    pkg_share = get_package_share_directory("rviz_sim")
    urdf_file_path = os.path.join(pkg_share, "urdf", LaunchConfiguration("urdf_file").perform(context))

    # Read URDF file
    if os.path.exists(urdf_file_path):
        with open(urdf_file_path, "r") as infp:
            robot_description_content = infp.read()
    else:
        raise RuntimeError(f"URDF file not found: {urdf_file_path}")

    robot_description = ParameterValue(robot_description_content, value_type=str)

    # RViz node
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(LaunchConfiguration("rviz_enable")),
        arguments=[
            "-d",
            os.path.join(pkg_share, "launch", "display_ros2_v2.rviz"),
            "--ros-args",
            "--log-level",
            "warn",
        ],
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return [node_rviz, node_robot_state_publisher]


def generate_launch_description():
    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
