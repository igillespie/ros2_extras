from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='rover_arm').find('rover_arm')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'rover_arm.urdf.xacro'])
    config_file = PathJoinSubstitution([pkg_share, 'config', 'rover_arm_controllers.yaml'])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
            output='screen',
            #arguments=['--ros-args', '--log-level', 'debug'],  # Enable debug logs
        ),
        # ROS 2 Control Node
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': Command(['xacro ', urdf_file])},
                config_file,
            ],
        ),
        # Spawner Nodes
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
        ),
    ])