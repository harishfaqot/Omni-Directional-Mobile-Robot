import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction,
    DeclareLaunchArgument, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from os.path import join


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('robotpadi_description')

    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'robotpadi.xacro')
    ros_gz_bridge_config   = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')
    rviz_config_file       = os.path.join(pkg_ros_gz_rbot, 'config', 'gazebo.rviz')
    world_file             = os.path.join(pkg_ros_gz_rbot, 'worlds', 'paddy_generated.world')

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    pkg_parent = os.path.dirname(pkg_ros_gz_rbot)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            pkg_ros_gz_rbot,          # share/robotpadi_description  (for worlds/, models/ inside package)
            pkg_parent,               # share/  (so ../models/ from worlds/ resolves here)
        ])
    )

    # ── Nodes ────────────────────────────────────────────────────────────────

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[('/joint_states', '/joint_states')],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -v 4 --render-engine ogre {world_file}'
        }.items()
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name', 'robotpadi',
                '-allow_renaming', 'false',
                '-x', '-5.0',
                '-y', '-5.0',
                '-z', '0.35',
                '-Y', '0.0',
            ],
            output='screen'
        )]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': ros_gz_bridge_config}],
        output='screen'
    )

    load_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )]
    )

    load_steering_controller = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['steering_controller'],
            output='screen',
        )]
    )

    load_wheel_controller = TimerAction(
        period=10.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['wheel_controller'],
            output='screen',
        )]
    )

    rviz_node = TimerAction(
        period=6.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )]
    )

    swerve_nav_goal = TimerAction(
        period=12.0,
        actions=[Node(
            package='robotpadi_controller',
            executable='swerve_nav_goal',
            name='swerve_nav_goal',
            output='screen',
        )]
    )

    return LaunchDescription([
        set_gz_resource_path,       # <-- must come first, before gazebo starts
        robot_state_publisher,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        load_joint_state_broadcaster,
        load_steering_controller,
        load_wheel_controller,
        rviz_node,
        swerve_nav_goal,
    ])